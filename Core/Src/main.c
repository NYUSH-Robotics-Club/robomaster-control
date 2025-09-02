/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  * This software is licensed as-is.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float target;
    float actual;
    float last_actual;

    float error;
    float last_error;
    float integral;

    float output;
    float output_max;
    float integral_max;
} PID_Controller;

typedef struct {
    uint16_t angle;
    int16_t  speed;
    int16_t  current;
    uint8_t  temp;
    uint32_t last_update_time;
} Motor_Feedback;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 底盘的四个电机
#define MOTOR_COUNT                      (4)
#define MOTOR_STDID_1_4                 (0x200U)
#define MOTOR_STDID_5_8                 (0x1FFU)

// 等待电调自动！
#define WAIT_ESC_BOOT_MS                (500U)
// while循环中的刷新周期
#define CMD_REFRESH_INTERVAL_MS         (5U)
// 电机反馈超时时间，电调没有更新就认为电机没有运行，直接输出0
#define MOTOR_FEEDBACK_TIMEOUT_MS       (100U)
// 神奇小按钮的抖动时间
#define KEY_DEBOUNCE_MS                 (200U)
// 电机自动运行时间
#define MOTOR_AUTO_RUN_TIME_MS          (4000U)

// 目标速度，这个是转子的速度，1:19减速比，所以转子最高速度理论上是9000rpm
#define DEMO_TARGET_SPEED               (7000)
// 加速度
#define RAMP_STEP                       (50.0f)

// 比例，kp决定当前值逼近目标值的速度，越大响应越快，但是可能靠近就boom
#define SPEED_PID_KP                    (5.0f)
// 积分，ki决定对误差的累积，越大消除稳态误差就越快
#define SPEED_PID_KI                    (0.5f)
// 微分，kd决定对变化速度的抑制力度，越大抑制越强，有个阻尼的效果
#define SPEED_PID_KD                    (0.1f)

// 输出最大值和积分最大值
#define SPEED_PID_OUTPUT_MAX            (15000)
#define SPEED_PID_INTEGRAL_MAX          (7500)

// C板上的CAN1配置，C板通过CAN连电调中心板，再通过中心板连各个电调
#define CAN1_TIMING_PRESCALER           (3U)
#define CAN1_TIMING_BS1                 (CAN_BS1_11TQ)
#define CAN1_TIMING_BS2                 (CAN_BS2_2TQ)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
Motor_Feedback motor_feedbacks[8];
PID_Controller speed_pids[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
static void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float output_max, float integral_max);
static float PID_Calculate(PID_Controller *pid, float target, float actual);
static void RGB_Init(void);
static void LED_SetRGB(uint8_t r, uint8_t g, uint8_t b);
static void CAN1_StartAll(void);
static HAL_StatusTypeDef M3508_SendCurrent4(int16_t i1,int16_t i2,int16_t i3,int16_t i4,uint16_t stdId);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
初始化PID控制器
*/
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float output_max, float integral_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->output_max = output_max;
    pid->integral_max = integral_max;
    pid->target = 0.0f;
    pid->actual = 0.0f;
    pid->last_actual = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

/*
计算PID输出
*/
float PID_Calculate(PID_Controller *pid, float target, float actual)
{
    pid->target = target;
    pid->actual = actual;
    pid->error = pid->target - pid->actual;

    pid->integral += pid->error;
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }

    
    float derivative = - (pid->actual - pid->last_actual);
    pid->output = pid->Kp * pid->error + pid->Ki * pid->integral + pid->Kd * derivative;

    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < -pid->output_max) {
        pid->output = -pid->output_max;
    }

    pid->last_error = pid->error;
    pid->last_actual = pid->actual;
    return pid->output;
}

/*
启动CAN1
*/
static void CAN1_StartAll(void)
{
  CAN_FilterTypeDef f = {0};
  f.FilterActivation      = ENABLE;
  f.FilterMode            = CAN_FILTERMODE_IDMASK;
  f.FilterScale           = CAN_FILTERSCALE_32BIT;
  f.FilterFIFOAssignment  = CAN_FILTER_FIFO0;
  f.FilterIdHigh          = 0x0000;
  f.FilterIdLow           = 0x0000;
  f.FilterMaskIdHigh      = 0x0000;
  f.FilterMaskIdLow       = 0x0000;
  HAL_CAN_ConfigFilter(&hcan1, &f);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/*
发送电流到电调
*/
static HAL_StatusTypeDef M3508_SendCurrent4(int16_t i1,int16_t i2,int16_t i3,int16_t i4,uint16_t stdId)
{
  CAN_TxHeaderTypeDef tx = {0};
  uint8_t d[8];
  uint32_t mb;

  tx.StdId = stdId;
  tx.IDE   = CAN_ID_STD;
  tx.RTR   = CAN_RTR_DATA;
  tx.DLC   = 8;

  d[0] = (uint8_t)(i1 >> 8); d[1] = (uint8_t)i1;
  d[2] = (uint8_t)(i2 >> 8); d[3] = (uint8_t)i2;
  d[4] = (uint8_t)(i3 >> 8); d[5] = (uint8_t)i3;
  d[6] = (uint8_t)(i4 >> 8); d[7] = (uint8_t)i4;

  return HAL_CAN_AddTxMessage(&hcan1, &tx, d, &mb);
}

/*
CAN1接收回调
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx;
  uint8_t d[8];

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, d) != HAL_OK) return;

  if (rx.IDE==CAN_ID_STD && rx.DLC==8 && rx.StdId>=0x201 && rx.StdId<=0x208) {
    uint8_t  mid   = rx.StdId - 0x201;
    if (mid < 8) {
        motor_feedbacks[mid].angle = (d[0]<<8) | d[1];
        motor_feedbacks[mid].speed = (int16_t)((d[2]<<8) | d[3]);
        motor_feedbacks[mid].current = (int16_t)((d[4]<<8) | d[5]);
        motor_feedbacks[mid].temp = d[6];
        motor_feedbacks[mid].last_update_time = HAL_GetTick();
    }
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
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_CAN1_Init();

  /* USER CODE BEGIN 2 */
  LED_SetRGB(1,1,1); HAL_Delay(200);

  CAN1_StartAll();
  HAL_Delay(WAIT_ESC_BOOT_MS);

  
  PID_Init(&speed_pids[0], SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
  PID_Init(&speed_pids[1], SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
  PID_Init(&speed_pids[2], SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
  PID_Init(&speed_pids[3], SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
  

  uint32_t initial_tick = HAL_GetTick();
  for (int i = 0; i < 8; i++) {
      motor_feedbacks[i].last_update_time = initial_tick;
  }

  bool motor_running = false;
  uint32_t last_key_press_time = 0;
  uint32_t motor_start_time = 0;
  bool motor_auto_stop_enabled = false;
  int16_t output_currents[MOTOR_COUNT] = {0};
  float ramped_target_speed = 0.0f;
  
  // 1和4：正向旋转
  // 2和3：反向旋转
  float motor_target_speeds[MOTOR_COUNT] = {DEMO_TARGET_SPEED, -DEMO_TARGET_SPEED, -DEMO_TARGET_SPEED, DEMO_TARGET_SPEED};
  float ramped_motor_targets[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f}; // 初始化为0

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    uint32_t current_tick = HAL_GetTick();


    // 按钮控制电机的逻辑
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
    {
        if (current_tick - last_key_press_time > KEY_DEBOUNCE_MS)
        {
            last_key_press_time = current_tick;
            
            if (!motor_running)
            {
                // 启动电机
                motor_running = true;
                motor_start_time = current_tick;
                motor_auto_stop_enabled = true;
            }
            else
            {
                // 手动停止电机
                motor_running = false;
                motor_auto_stop_enabled = false;
                for (int i = 0; i < MOTOR_COUNT; i++) {
                    speed_pids[i].integral = 0.0f;
                }
            }
        }
    }

    // 检查是否达到自动停止时间
    if (motor_running && motor_auto_stop_enabled && (current_tick - motor_start_time >= MOTOR_AUTO_RUN_TIME_MS))
    {
        motor_running = false;
        motor_auto_stop_enabled = false;
        for (int i = 0; i < MOTOR_COUNT; i++) {
            speed_pids[i].integral = 0.0f;
        }
    }


    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        if (motor_running)
        {
            if (motor_target_speeds[i] > 0)
            {
                // 正向电机
                if (ramped_motor_targets[i] < motor_target_speeds[i])
                {
                    ramped_motor_targets[i] += RAMP_STEP;
                    if (ramped_motor_targets[i] > motor_target_speeds[i])
                    {
                        ramped_motor_targets[i] = motor_target_speeds[i];
                    }
                }
            }
            else
            {
                // 反向电机
                if (ramped_motor_targets[i] > motor_target_speeds[i])
                {
                    ramped_motor_targets[i] -= RAMP_STEP;
                    if (ramped_motor_targets[i] < motor_target_speeds[i])
                    {
                        ramped_motor_targets[i] = motor_target_speeds[i];
                    }
                }
            }
        }
        else
        {
            // 停止时逐渐减速到0
            if (ramped_motor_targets[i] > 0)
            {
                ramped_motor_targets[i] -= RAMP_STEP;
                if (ramped_motor_targets[i] < 0)
                {
                    ramped_motor_targets[i] = 0;
                }
            }
            else if (ramped_motor_targets[i] < 0)
            {
                ramped_motor_targets[i] += RAMP_STEP;
                if (ramped_motor_targets[i] > 0)
                {
                    ramped_motor_targets[i] = 0;
                }
            }
        }
    }


    // 计算输出电流
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        // 如果电调没有更新，就认为电机没有运行，直接输出0
        if (current_tick - motor_feedbacks[i].last_update_time > MOTOR_FEEDBACK_TIMEOUT_MS)
        {
            output_currents[i] = 0;
        }
        else
        {
            // 根据电调的反馈计算输出电流，使用每个电机的独立目标速度
            float current_speed = motor_feedbacks[i].speed;
            output_currents[i] = (int16_t)PID_Calculate(&speed_pids[i], ramped_motor_targets[i], current_speed);
        }
    }

    bool any_motor_running = false;
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        if (ramped_motor_targets[i] != 0)
        {
            any_motor_running = true;
            break;
        }
    }
    
    if (any_motor_running)
    {
        LED_SetRGB(0, 1, 0); // 绿色：电机运行中
    }
    else
    {
        LED_SetRGB(0, 0, 1); // 蓝色：电机停止
    }

    // 最后发送输出电流
    M3508_SendCurrent4(output_currents[0], output_currents[1], output_currents[2], output_currents[3], MOTOR_STDID_1_4);
    
    int16_t other_currents[4] = {1000, -10000, 10000, 0};
    M3508_SendCurrent4(other_currents[0], other_currents[1], other_currents[2], other_currents[3], MOTOR_STDID_5_8);
    HAL_Delay(CMD_REFRESH_INTERVAL_MS);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = CAN1_TIMING_PRESCALER;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN1_TIMING_BS1;
  hcan1.Init.TimeSeg2 = CAN1_TIMING_BS2;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


/* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin   = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
设置灯光
*/
static void LED_SetRGB(uint8_t r, uint8_t g, uint8_t b)
{
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, r ? GPIO_PIN_SET : GPIO_PIN_RESET); // R
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, g ? GPIO_PIN_SET : GPIO_PIN_RESET); // G
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, b ? GPIO_PIN_SET : GPIO_PIN_RESET); // B
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

#ifdef  USE_FULL_ASSERT
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