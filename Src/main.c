/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmi088driver.h"
#include "buzzer.h"
#include "usbd_cdc_if.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "remote_control.h"
#include "chassis_controller.h"
#include "shooter_controller.h"
#include "can_manager.h"
#include <stdarg.h>
#include "printing.h"
#include "wt61c.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Wait for ESC boot
#define WAIT_ESC_BOOT_MS                (500U)
// Main loop refresh interval
#define CMD_REFRESH_INTERVAL_MS         (5U)
// Debug info interval
#define USB_DEBUG_INTERVAL_MS           (1000U)
// RC loss timeout for health gating
#define RC_LOSS_TIMEOUT_MS              (200U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Controller instances
ChassisController chassis_controller;
ShooterController shooter_controller;

// CAN managers
CAN_Manager_t can1_manager;
CAN_Manager_t can2_manager;

// USB CDC variables
static uint32_t last_debug_time = 0;
static uint32_t last_frame_count = 0;
// RC health tracking
static uint32_t last_rc_tick = 0;
static uint32_t last_rc_fc = 0;
// RC baseline and sanitized view
static int16_t rc_baseline[5] = {0};
static uint8_t rc_baseline_set = 0;
static RC_ctrl_t rc_sanitized;

float gyro[3], accel[3], temp;

// WT61C-TTL IMU sensor on USART1
#define WT61C_UART_HANDLE  huart1
#define RX_DMA_BUF_SZ 256
static uint8_t wt61c_rxbuf[RX_DMA_BUF_SZ];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void LED_SetRGB(uint8_t r, uint8_t g, uint8_t b);

SensorData sensor_data;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
CAN receive callback - delegate to CAN manager
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_Manager_GlobalCallback(hcan);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // Optional: initial tiny transmission (currently disabled)
  // USB_CDC_SendString("\r\n");

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  BMI088_init();
  // Initialize DT7/DBUS receiver on USART3 + DMA double buffer
  // Used for remote control
  remote_control_init();

  // Initialize CAN
  CAN_Manager_Init(&can1_manager, CAN_CHANNEL_1, &hcan1, &chassis_controller, &shooter_controller);
  CAN_Manager_Init(&can2_manager, CAN_CHANNEL_2, &hcan2, &chassis_controller, &shooter_controller);

  // Start CAN communication
  CAN_Manager_Start(&can1_manager);
  CAN_Manager_Start(&can2_manager);

  HAL_Delay(WAIT_ESC_BOOT_MS);

  // Initialize motor controllers
  ChassisController_Init(&chassis_controller);
  ShooterController_Init(&shooter_controller);

  // Play boot beep sound
  Buzzer_PlayBeep();

  // Ensure system is ready after boot song
  HAL_Delay(100);

  // Initialize WT61C-TTL IMU sensor on USART1
  WT61C_Init(&WT61C_UART_HANDLE);
  // Start UART DMA reception with idle line detection
  HAL_UARTEx_ReceiveToIdle_DMA(&WT61C_UART_HANDLE, wt61c_rxbuf, RX_DMA_BUF_SZ);
  // Disable half-transfer interrupt to reduce callback overhead
  __HAL_DMA_DISABLE_IT(WT61C_UART_HANDLE.hdmarx, DMA_IT_HT);

  // Optional: Configure WT61C for 100Hz output rate
  // HAL_Delay(50);
  // WT61C_Unlock(&WT61C_UART_HANDLE);
  // HAL_Delay(10);
  // WT61C_SetReturnRate(&WT61C_UART_HANDLE, 0x09); // 100Hz
  // HAL_Delay(10);
  // WT61C_Save(&WT61C_UART_HANDLE);
  // HAL_Delay(10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	uint32_t current_tick = HAL_GetTick();
	const RC_ctrl_t *raw_rc = get_remote_control_point();
	// RC health gating based on frame count activity
	uint32_t fc_now = RC_GetFrameCount();

  //read the IMU data from the C board
  BMI088_read(gyro, accel, &temp);
  static uint32_t last_output_time = 0;
  static uint32_t frame_count = 0;
  uint32_t now = HAL_GetTick();

  frame_count++;

  // Output data only every 100ms to avoid overflow
  if (now - last_output_time > 100) {
      USB_CDC_Printf("GYRO: %d, %d, %d\r\n", (int)gyro[0], (int)gyro[1], (int)gyro[2]);
      last_output_time = now;
  }

	if (fc_now != last_rc_fc)
	{
		last_rc_fc = fc_now;
		last_rc_tick = current_tick;
	}
	bool rc_healthy = (current_tick - last_rc_tick) <= RC_LOSS_TIMEOUT_MS;
	const RC_ctrl_t *rc_data = NULL;
	if (rc_healthy && raw_rc)
	{
		if (!rc_baseline_set)
		{
			for (int i = 0; i < 5; ++i) rc_baseline[i] = raw_rc->rc.ch[i];
			rc_baseline_set = 1;
		}
		// Build sanitized view by subtracting baseline
		rc_sanitized = *raw_rc;
		for (int i = 0; i < 5; ++i)
		{
			int32_t v = (int32_t)raw_rc->rc.ch[i] - (int32_t)rc_baseline[i];
			// optional clamp to reasonable range
			if (v > 660) v = 660; else if (v < -660) v = -660;
			rc_sanitized.rc.ch[i] = (int16_t)v;
		}
		rc_data = &rc_sanitized;
	}

	// Update controllers
	ChassisController_Update(&chassis_controller, rc_data, current_tick, sensor_data);
	ShooterController_Update(&shooter_controller, rc_data, current_tick);
	
	// Update buzzer music playback
	Buzzer_Update();

	// Compute and send motor currents
	ChassisController_ComputeCurrents(&chassis_controller, current_tick);
	ShooterController_ComputeCurrents(&shooter_controller, current_tick);

	// LED status indication
	bool chassis_running = ChassisController_IsRunning(&chassis_controller);
	bool shooter_running = ShooterController_IsRunning(&shooter_controller);
	if (chassis_running || shooter_running)
	{
		LED_SetRGB(0, 1, 0); // Green when running
	}
	else
	{
		LED_SetRGB(1, 0, 1); // Red when stopped
	}

  // Periodic debug output
  if (current_tick - last_debug_time >= USB_DEBUG_INTERVAL_MS)
  {
    last_debug_time = current_tick;
  uint32_t fc = RC_GetFrameCount();
  (void)last_frame_count; // suppress unused if RC debug disabled

    // Debug_PrintCANStatus(current_tick);  // disabled while focusing on RC
    // Debug_PrintCANDiag();                // disabled while focusing on RC

    // RC and control path quick diagnostics
    {
      // Print sanitized channels (after baseline removal)
      // int16_t ch0 = 0, ch2 = 0, ch3 = 0, ch4 = 0; uint8_t swl = 0;
      // if (rc_data) { ch0 = rc_data->rc.ch[0]; ch2 = rc_data->rc.ch[2]; ch3 = rc_data->rc.ch[3]; ch4 = rc_data->rc.ch[4]; swl = (uint8_t)rc_data->rc.s[0]; }
      // USB_CDC_Printf("RC fc=%lu ch0=%d ch2=%d ch3=%d ch4=%d swL=%u\r\n",
      //   (unsigned long)fc, (int)ch0, (int)ch2, (int)ch3, (int)ch4, (unsigned int)swl);
      // if (rc_baseline_set)
      // {
      //   USB_CDC_Printf("RC baseline=[%d,%d,%d,%d,%d]\r\n",
      //     (int)rc_baseline[0], (int)rc_baseline[1], (int)rc_baseline[2], (int)rc_baseline[3], (int)rc_baseline[4]);
      // }

      // Centering and deadband check for chassis channels
      // const int16_t deadband = 10;
      // bool c0 = (ch0 > -deadband && ch0 < deadband);
      // bool c2 = (ch2 > -deadband && ch2 < deadband);
      // bool c3 = (ch3 > -deadband && ch3 < deadband);
      // USB_CDC_Printf("RC centered: ch0=%d ch2=%d ch3=%d\r\n", c0?1:0, c2?1:0, c3?1:0);

      // Dump last SBUS frame bytes for mapping investigation
      // uint8_t sbus_dump[RC_FRAME_LENGTH];
      // memset(sbus_dump, 0, sizeof(sbus_dump));
      // RC_GetLastFrame(sbus_dump);

      // USB_CDC_Printf("SBUS:");
      // for (int i = 0; i < (int)RC_FRAME_LENGTH; ++i) {
      //   USB_CDC_Printf(" %02X", (unsigned int)sbus_dump[i]);
      // }
      // USB_CDC_Printf("\r\n");

      // bool ch_run = ChassisController_IsRunning(&chassis_controller);
      // bool sh_run = ShooterController_IsRunning(&shooter_controller);
      
      // USB_CDC_Printf("RUN ch=%d sh=%d\r\n", ch_run?1:0, sh_run?1:0);

      // DISABLED: BMI088 data conflicts with WT61C output
      // USB_CDC_Printf("GYRO [%.2f, %.2f, %.2f] ACCEL [%.2f, %.2f, %.2f] TEMP %.2f\r\n",
      //   gyro[0], gyro[1], gyro[2],
      //   accel[0], accel[1], accel[2],
      //   temp);

      

      // Chassis targets and outputs
      // USB_CDC_Printf("CH tgt=[%d,%d,%d,%d] out=[%d,%d,%d,%d]\r\n",
      //   (int)chassis_controller.ramped_targets[0],
      //   (int)chassis_controller.ramped_targets[1],
      //   (int)chassis_controller.ramped_targets[2],
      //   (int)chassis_controller.ramped_targets[3],
      //   (int)chassis_controller.output_currents[0],
      //   (int)chassis_controller.output_currents[1],
      //   (int)chassis_controller.output_currents[2],
      //   (int)chassis_controller.output_currents[3]);

      // // Shooter outputs and gimbal current
      // // const int16_t *sh_out = ShooterController_GetOutputCurrents(&shooter_controller);
      // // int16_t sh0 = 0, sh1 = 0, sh2 = 0, sh3 = 0;
      // // if (sh_out) { sh0 = sh_out[0]; sh1 = sh_out[1]; sh2 = sh_out[2]; sh3 = sh_out[3]; }
      // // USB_CDC_Printf("SH out=[%d,%d,%d,%d]\r\n", (int)sh0, (int)sh1, (int)sh2, (int)sh3);
   

      

    }
    last_frame_count = fc; // kept to avoid large delta when re-enabled
  }

	HAL_Delay(CMD_REFRESH_INTERVAL_MS);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*
Set LED
*/
static void LED_SetRGB(uint8_t r, uint8_t g, uint8_t b)
{
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, r ? GPIO_PIN_SET : GPIO_PIN_RESET); // R
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, g ? GPIO_PIN_SET : GPIO_PIN_RESET); // G
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, b ? GPIO_PIN_SET : GPIO_PIN_RESET); // B
}

/*
UART DMA/Idle callback for WT61C sensor data reception
*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &WT61C_UART_HANDLE) {
    // Process received data
    WT61C_ProcessBytes(wt61c_rxbuf, Size);
    // Restart DMA reception
    HAL_UARTEx_ReceiveToIdle_DMA(&WT61C_UART_HANDLE, wt61c_rxbuf, RX_DMA_BUF_SZ);
    __HAL_DMA_DISABLE_IT(WT61C_UART_HANDLE.hdmarx, DMA_IT_HT);
  }
}

/*
WT61C new data callback - sends JSON formatted data via USB CDC
*/
void WT61C_OnNewData(const WT61C_Data *d)
{
  // Throttle output heavily to avoid USB buffer overflow
  static uint32_t last_output_time = 0;
  static uint32_t frame_count = 0;
  uint32_t now = HAL_GetTick();

  frame_count++;

  // Output data only every 100ms to avoid overflow
  if (now - last_output_time < 100) {
    return; // Skip this update
  }
  last_output_time = now;

  // Convert floats to integers for printf (workaround for missing float support)
  int ax_i = sensor_data.ax = (int)(d->ax * 1000);  // m/s^2 * 1000
  int ay_i = sensor_data.ay = (int)(d->ay * 1000);
  int az_i = sensor_data.az = (int)(d->az * 1000);
  int gx_i = sensor_data.gx = (int)(d->gx * 10);    // deg/s * 10
  int gy_i = sensor_data.gy = (int)(d->gy * 10);
  int gz_i = sensor_data.gz = (int)(d->gz * 10);
  int roll_i = sensor_data.roll = (int)(d->roll * 10);
  int pitch_i = sensor_data.pitch = (int)(d->pitch * 10);
  int yaw_i = sensor_data.yaw = (int)(d->yaw * 10);
  int temp_i = (int)(d->temperature * 10);

  // USB_CDC_Printf("{\"ax\":%d,\"ay\":%d,\"az\":%d,"
  //                "\"gx\":%d,\"gy\":%d,\"gz\":%d,"
  //                "\"roll\":%d,\"pitch\":%d,\"yaw\":%d,"
  //                "\"T\":%d}\r\n",
  //                (int)ax_i, (int)ay_i, (int)az_i,
  //                (int)gx_i, (int)gy_i, (int)gz_i,
  //                (int)roll_i, (int)pitch_i, (int)yaw_i,
  //                (int)temp_i);

  USB_CDC_Printf("\"gz\":%d", (int)gz_i, "\n");


}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
