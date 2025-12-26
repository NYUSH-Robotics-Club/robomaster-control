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
#include "gimbal_controller.h"
#include "gm6020_motor.h"
#include "can_manager.h"
#include <stdarg.h>
#include <math.h>
#include "printing.h"
#include "wt61c.h"
#include "gyro_data.h"
#include "message_center.h"
#include "app_subscriptions.h"
#include "cmd_controller.h"
#include "vision_comm.h"

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
// RC loss timeout for health gating
#define RC_LOSS_TIMEOUT_MS              (200U)
// USART6 hello message send interval
#define USART6_SEND_INTERVAL_MS         (1000U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// CAN managers
CAN_Manager_t can1_manager;
CAN_Manager_t can2_manager;

float gyro[3], accel[3], temp;

// WT61C-TTL IMU sensor on USART1
#define WT61C_UART_HANDLE  huart1
#define RX_DMA_BUF_SZ 256
static uint8_t wt61c_rxbuf[RX_DMA_BUF_SZ];

// Message center buffer
#define MSG_CENTER_QUEUE_LEN 128
static MsgEvent g_msg_queue[MSG_CENTER_QUEUE_LEN];

// USART6 periodic send timer
static uint32_t last_usart6_send_tick = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void LED_SetRGB(uint8_t r, uint8_t g, uint8_t b);
static void Gimbal_HoldPosition_Callback(void);

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

/**
 * @brief Callback function to hold gimbal position during calibration
 * @note This function is called every ~1ms during gyro calibration to keep
 *       the gimbal motors actively holding their position.
 */
static void Gimbal_HoldPosition_Callback(void)
{
  static uint32_t call_count = 0;
  call_count++;

  // Only send command and dispatch every 5ms to reduce overhead
  if (call_count % 5 == 0) {
    GimbalCmd cmd = {
      .enabled = true,
      .pitch_rate = 0.0f,
      .yaw_rate = 0.0f,
      .yaw_rate_memo = 0.0f,
      .yaw_target_memo = 0.0f,
      .vision_valid = false,
      .vision_yaw_err_rad = 0.0f,
      .vision_pitch_err_rad = 0.0f,
      .vision_ts_ms = 0
    };

    MsgCenter_Publish(TOPIC_GIMBAL_CMD, &cmd, sizeof(cmd));
    MsgCenter_Dispatch();  // Process messages immediately
  }
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
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  // Print boot message
  USB_CDC_Printf("\r\n");
  USB_CDC_Printf("========================================\r\n");
  USB_CDC_Printf("   RoboMaster Control System Boot\r\n");
  USB_CDC_Printf("========================================\r\n");

  // Initialize buzzer
  Buzzer_Init();

  // BMI088 Initialization (without calibration yet)
  USB_CDC_Printf("\r\n=== BMI088 Initialization ===\r\n");

  // Test CS pins
  USB_CDC_Printf("[BMI088] Testing CS pins...\r\n");
  USB_CDC_Printf("[BMI088] Testing ACCEL CS (PA4)...\r\n");
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_Delay(10);
  GPIO_PinState accel_cs_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
  USB_CDC_Printf("[BMI088] ACCEL CS HIGH: %s\r\n", accel_cs_state == GPIO_PIN_SET ? "OK" : "FAIL");

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_Delay(10);
  accel_cs_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
  USB_CDC_Printf("[BMI088] ACCEL CS LOW: %s\r\n", accel_cs_state == GPIO_PIN_RESET ? "OK" : "FAIL");
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  USB_CDC_Printf("[BMI088] Testing GYRO CS (PB0)...\r\n");
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_Delay(10);
  GPIO_PinState gyro_cs_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
  USB_CDC_Printf("[BMI088] GYRO CS HIGH: %s\r\n", gyro_cs_state == GPIO_PIN_SET ? "OK" : "FAIL");

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_Delay(10);
  gyro_cs_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
  USB_CDC_Printf("[BMI088] GYRO CS LOW: %s\r\n", gyro_cs_state == GPIO_PIN_RESET ? "OK" : "FAIL");
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  uint8_t bmi088_error = BMI088_init();
  USB_CDC_Printf("[BMI088] BMI088_init() returned: 0x%02X\r\n", bmi088_error);
  if (bmi088_error != 0) {
    USB_CDC_Printf("[BMI088] *** ERROR: BMI088 initialization failed! Error code: 0x%02X ***\r\n", bmi088_error);
    USB_CDC_Printf("[BMI088] Error details:\r\n");
    if (bmi088_error & 0x01) USB_CDC_Printf("  - BMI088_ACC_PWR_CTRL_ERROR\r\n");
    if (bmi088_error & 0x02) USB_CDC_Printf("  - BMI088_ACC_PWR_CONF_ERROR\r\n");
    if (bmi088_error & 0x04) USB_CDC_Printf("  - BMI088_ACC_CONF_ERROR\r\n");
    if (bmi088_error & 0x08) USB_CDC_Printf("  - BMI088_ACC_SELF_TEST_ERROR\r\n");
    if (bmi088_error & 0x10) USB_CDC_Printf("  - BMI088_ACC_RANGE_ERROR\r\n");
    if (bmi088_error & 0x20) USB_CDC_Printf("  - BMI088_INT1_IO_CTRL_ERROR\r\n");
    if (bmi088_error & 0x40) USB_CDC_Printf("  - BMI088_INT_MAP_DATA_ERROR\r\n");
    if (bmi088_error & 0x80) USB_CDC_Printf("  - GYRO initialization errors\r\n");
  } else {
    USB_CDC_Printf("[BMI088] BMI088 initialization SUCCESS!\r\n");
  }
  USB_CDC_Printf("=== BMI088 Init Complete ===\r\n\r\n");

  // Initialize message center (needed for gimbal communication)
  MsgCenter_Init(g_msg_queue, MSG_CENTER_QUEUE_LEN);

  // Initialize CAN managers early (needed for gimbal motors)
  CAN_Manager_Init(&can1_manager, CAN_CHANNEL_1, &hcan1);
  CAN_Manager_Init(&can2_manager, CAN_CHANNEL_2, &hcan2);
  CAN_Manager_Start(&can1_manager);
  CAN_Manager_Start(&can2_manager);

  // Initialize gimbal early (before calibration)
  GimbalApp_Init();

  // Wait for CAN bus to stabilize and gimbal to receive initial feedback
  USB_CDC_Printf("[Init] Waiting for CAN bus to stabilize...\r\n");
  HAL_Delay(200);

  // Wait for gimbal to reach initial alignment position
  Gimbal_WaitForAlignment();

  // Now start IMU calibration with gimbal in position
  USB_CDC_Printf("\r\n=== Starting IMU Calibration ===\r\n");
  USB_CDC_Printf("[Calibration] Keep the robot still!\r\n");

  // Set LED to blue during calibration
  LED_SetRGB(0, 0, 1);

  // Set callback to keep gimbal holding position during calibration
  gyro_calibrate_set_callback(Gimbal_HoldPosition_Callback);

  // Perform gyro calibration
  gyro_calibrate();

  // Set LED to green
  LED_SetRGB(0, 1, 0);

  USB_CDC_Printf("=== IMU Calibration Complete ===\r\n\r\n");

  // Continue with remaining initialization
  // Initialize command controller (central control)
  CmdController_Init();

  // Now we can clear the callback since CmdController will take over
  gyro_calibrate_set_callback(NULL);

  // Initialize remaining application controllers
  ChassisApp_Init();
  ShooterApp_Init();

  // Initialize remote control
  remote_control_init();

  // Initialize Vision Communication
  VisionComm_Init();

  // Wait for ESC boot
  HAL_Delay(WAIT_ESC_BOOT_MS);

  // Initialize WT61C-TTL IMU sensor on USART1
  WT61C_Init(&WT61C_UART_HANDLE);
  // Start UART DMA reception with idle line detection
  HAL_UARTEx_ReceiveToIdle_DMA(&WT61C_UART_HANDLE, wt61c_rxbuf, RX_DMA_BUF_SZ);
  // Disable half-transfer interrupt to reduce callback overhead
  __HAL_DMA_DISABLE_IT(WT61C_UART_HANDLE.hdmarx, DMA_IT_HT);

  USB_CDC_Printf("\r\n=== System Ready ===\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t current_tick = HAL_GetTick();

    // Update sensor data and publishes IMU topic
    gyro_data_update(&sensor_data);

    // Process command controller
    LED_SetRGB(1, 0, 0);
    CmdController_Task(current_tick);

    // Dispatch message center events
    MsgCenter_Dispatch();

    // Update buzzer music playback (feature for fun :D)
    Buzzer_Update();

    // Send "hello" through USART6 periodically
    if (current_tick - last_usart6_send_tick >= USART6_SEND_INTERVAL_MS)
    {
        USART6_SendString("hello\r\n");
        last_usart6_send_tick = current_tick;
    }

    LED_SetRGB(0, 1, 0);

	  HAL_Delay(CMD_REFRESH_INTERVAL_MS);

    /* USER CODE END WHILE */
  }
    /* USER CODE BEGIN 3 */
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
  else if (huart == &huart6) {
    // Process vision communication data (USART6 for vision system)
    extern void VisionComm_RxCallback(uint8_t *buf, uint32_t len);
    extern uint8_t uart_recv_buff[18];
    VisionComm_RxCallback(uart_recv_buff, Size);
  }
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
