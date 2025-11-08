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
#include "can_manager.h"
#include <stdarg.h>
#include "printing.h"
#include "wt61c.h"
#include "gyro_data.h"
#include "message_center.h"
#include "app_subscriptions.h"
#include "cmd_controller.h"

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
  MsgCenter_Init(g_msg_queue, MSG_CENTER_QUEUE_LEN);
  
  // Initialize command controller first (central control)
  CmdController_Init();
  
  // Initialize application controllers
  ChassisApp_Init();
  ShooterApp_Init();
  GimbalApp_Init();
  
  // Initialize remote control
  remote_control_init();

  // Initialize CAN managers (they will publish TOPIC_CAN_RX and TOPIC_MOTOR_FEEDBACK)
  CAN_Manager_Init(&can1_manager, CAN_CHANNEL_1, &hcan1);
  CAN_Manager_Init(&can2_manager, CAN_CHANNEL_2, &hcan2);
  CAN_Manager_Start(&can1_manager);
  CAN_Manager_Start(&can2_manager);
  
  HAL_Delay(WAIT_ESC_BOOT_MS);

  // Play boot beep sound
  Buzzer_PlayBeep();

  // Initialize WT61C-TTL IMU sensor on USART1
  WT61C_Init(&WT61C_UART_HANDLE);
  // Start UART DMA reception with idle line detection
  HAL_UARTEx_ReceiveToIdle_DMA(&WT61C_UART_HANDLE, wt61c_rxbuf, RX_DMA_BUF_SZ);
  // Disable half-transfer interrupt to reduce callback overhead
  __HAL_DMA_DISABLE_IT(WT61C_UART_HANDLE.hdmarx, DMA_IT_HT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t current_tick = HAL_GetTick();

    // Update sensor data and publishes IMU topic
    gyro_data_update(&sensor_data);

    // Process command controller
    CmdController_Task(current_tick);
    
    // Dispatch message center events
    MsgCenter_Dispatch();

    // Update buzzer music playback (feature for fun :D)
    Buzzer_Update();

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
