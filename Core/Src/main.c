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
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include "remote_control.h"
#include "pid.h"
#include "gm6020.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
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

// 底盘速度，M3508，这个是转子的速度，1:19减速比，所以转子最高速度理论上是9000rpm
#define DEMO_TARGET_SPEED               (7000)
// 转盘，M2006
#define MOTOR5_CONST_SPEED              (3000)
// 加速度
#define RAMP_STEP                       (50.0f)
// 射击轮，M3508
#define SHOOTER_CONST_SPEED             (7500)

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

/* USER CODE BEGIN PV */
Motor_Feedback motor_feedbacks[8];
PID_Controller speed_pids[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void RGB_Init(void);
static void LED_SetRGB(uint8_t r, uint8_t g, uint8_t b);
static void CAN1_StartAll(void);
static HAL_StatusTypeDef CAN_SendMotorCurrents4(int16_t i1,int16_t i2,int16_t i3,int16_t i4,uint16_t stdId);
static void CAN2_StartAll(void);
static HAL_StatusTypeDef CAN_SendMotorCurrents4Ex(CAN_HandleTypeDef *hcan,
                                                  uint16_t stdId,
                                                  int16_t i1,int16_t i2,int16_t i3,int16_t i4);
static float RampTowards(float current, float target, float step);
static void ResetPidIntegralsRange(PID_Controller *pids, int start_idx, int count);
static void ComputeChassisCurrents(int16_t out_currents[MOTOR_COUNT], const float ramp_targets[MOTOR_COUNT], PID_Controller pids[8], Motor_Feedback feedbacks[8], uint32_t current_tick);
static int16_t ComputeSingleMotorCurrent(PID_Controller *pid, float target, Motor_Feedback *feedback, uint32_t current_tick);
static HAL_StatusTypeDef GM6020_SendCurrentById(uint8_t id, int16_t cur);
int16_t gm6020_control_from_joystick(uint8_t id, int16_t joystick_ch1);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static float RampTowards(float current, float target, float step)
{
    if (current < target)
    {
        current += step;
        if (current > target)
        {
            current = target;
        }
    }
    else if (current > target)
    {
        current -= step;
        if (current < target)
        {
            current = target;
        }
    }
    return current;
}

static void ResetPidIntegralsRange(PID_Controller *pids, int start_idx, int count)
{
    for (int i = 0; i < count; i++)
    {
        pids[start_idx + i].integral = 0.0f;
    }
}

static int16_t ComputeSingleMotorCurrent(PID_Controller *pid, float target, Motor_Feedback *feedback, uint32_t current_tick)
{
    if (current_tick - feedback->last_update_time > MOTOR_FEEDBACK_TIMEOUT_MS)
    {
        return 0;
    }
    float current_speed = feedback->speed;
    return (int16_t)PID_Calculate(pid, target, current_speed);
}

static void ComputeChassisCurrents(int16_t out_currents[MOTOR_COUNT], const float ramp_targets[MOTOR_COUNT], PID_Controller pids[8], Motor_Feedback feedbacks[8], uint32_t current_tick)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        out_currents[i] = ComputeSingleMotorCurrent(&pids[i], ramp_targets[i], &feedbacks[i], current_tick);
    }
}

static HAL_StatusTypeDef GM6020_SendCurrentById(uint8_t id, int16_t cur)
{
  if (id < 1 || id > 7) return HAL_ERROR;

  if (cur >  30000) cur =  30000;
  if (cur < -30000) cur = -30000;

  uint16_t stdId = (id <= 4) ? 0x1FF : 0x2FF;
  uint8_t  slot  = (id <= 4) ? (uint8_t)(id - 1) : (uint8_t)(id - 5);

  int16_t i1 = 0, i2 = 0, i3 = 0, i4 = 0;
  switch (slot)
  {
    case 0: i1 = cur; break;
    case 1: i2 = cur; break;
    case 2: i3 = cur; break;
    case 3: i4 = cur; break;
    default: break;
  }

  return CAN_SendMotorCurrents4Ex(&hcan1, stdId, i1, i2, i3, i4);
}


/*
启动CAN1
*/
static void CAN1_StartAll(void)
{
  CAN_FilterTypeDef f = {0};
  f.FilterBank          = 0;
  f.SlaveStartFilterBank  = 14;
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

static void CAN2_StartAll(void)
{
  CAN_FilterTypeDef f = {0};
  f.FilterBank            = 14;
  f.FilterMode            = CAN_FILTERMODE_IDMASK;
  f.FilterScale           = CAN_FILTERSCALE_32BIT;
  f.FilterFIFOAssignment  = CAN_FILTER_FIFO0;
  f.FilterIdHigh          = 0x0000;
  f.FilterIdLow           = 0x0000;
  f.FilterMaskIdHigh      = 0x0000;
  f.FilterMaskIdLow       = 0x0000;
  f.FilterActivation      = ENABLE;
  

  HAL_CAN_ConfigFilter(&hcan2, &f);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

static HAL_StatusTypeDef CAN_SendMotorCurrents4Ex(CAN_HandleTypeDef *hcan,
                                                  uint16_t stdId,
                                                  int16_t i1,int16_t i2,int16_t i3,int16_t i4)
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

  return HAL_CAN_AddTxMessage(hcan, &tx, d, &mb);
}

/*
CAN接收回调
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx;
  uint8_t d[8];

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, d) != HAL_OK) return;
  if (hcan == &hcan1 && rx.IDE==CAN_ID_STD && rx.DLC==8 && rx.StdId>=0x205 && rx.StdId<=0x20B) {
    uint8_t gid = (uint8_t)(rx.StdId - 0x204);
    if (gid >= 1 && gid <= 7) {
        uint16_t angle_raw = (uint16_t)((d[0]<<8) | d[1]);
        int16_t  speed_rpm = (int16_t)((d[2]<<8) | d[3]);
        gm6020_on_feedback(gid, angle_raw, speed_rpm);
    }
  }
  else if (rx.IDE==CAN_ID_STD && rx.DLC==8 && rx.StdId>=0x201 && rx.StdId<=0x208) {
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // Initialize DT7/DBUS receiver on USART3 + DMA double buffer
  remote_control_init();

  CAN1_StartAll();
  CAN2_StartAll();
  HAL_Delay(WAIT_ESC_BOOT_MS);

  
  PID_Init(&speed_pids[0], SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
  PID_Init(&speed_pids[1], SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
  PID_Init(&speed_pids[2], SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
  PID_Init(&speed_pids[3], SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
  PID_Init(&speed_pids[4], SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
  PID_Init(&speed_pids[5], SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
  PID_Init(&speed_pids[6], SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
  gm6020_init(7);

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
  float ramped_motor5_target = 0.0f;
  float ramped_shooter1_target = 0.0f;
  float ramped_shooter2_target = 0.0f;
  int16_t output_currents_5_8[4] = {0};
  float motor_target_speeds[MOTOR_COUNT] = {DEMO_TARGET_SPEED, -DEMO_TARGET_SPEED, -DEMO_TARGET_SPEED, DEMO_TARGET_SPEED};
  float ramped_motor_targets[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
                ResetPidIntegralsRange(speed_pids, 0, MOTOR_COUNT);
                ResetPidIntegralsRange(speed_pids, 4, 3);
            }
        }
    }

    // 检查是否达到自动停止时间
    if (motor_running && motor_auto_stop_enabled && (current_tick - motor_start_time >= MOTOR_AUTO_RUN_TIME_MS))
    {
        motor_running = false;
        motor_auto_stop_enabled = false;
        ResetPidIntegralsRange(speed_pids, 0, MOTOR_COUNT);
        ResetPidIntegralsRange(speed_pids, 4, 3);
    }


    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        float target = motor_running ? motor_target_speeds[i] : 0.0f;
        ramped_motor_targets[i] = RampTowards(ramped_motor_targets[i], target, RAMP_STEP);
    }


    // 转盘
    ramped_motor5_target = RampTowards(ramped_motor5_target, motor_running ? MOTOR5_CONST_SPEED : 0.0f, RAMP_STEP);

    // 射击电机
    float shooter1_target = motor_running ? -SHOOTER_CONST_SPEED : 0.0f;
    float shooter2_target = motor_running ?  SHOOTER_CONST_SPEED : 0.0f;
    ramped_shooter1_target = RampTowards(ramped_shooter1_target, shooter1_target, RAMP_STEP);
    ramped_shooter2_target = RampTowards(ramped_shooter2_target, shooter2_target, RAMP_STEP);

    ComputeChassisCurrents(output_currents, ramped_motor_targets, speed_pids, motor_feedbacks, current_tick);

    int16_t motor5_current = ComputeSingleMotorCurrent(&speed_pids[4], ramped_motor5_target, &motor_feedbacks[4], current_tick);
    
    // 射击电机
    int16_t motor6_current = ComputeSingleMotorCurrent(&speed_pids[5], ramped_shooter1_target, &motor_feedbacks[5], current_tick);
    int16_t motor8_current = ComputeSingleMotorCurrent(&speed_pids[6], ramped_shooter2_target, &motor_feedbacks[6], current_tick);

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
        LED_SetRGB(0, 1, 0);
    }else{
        LED_SetRGB(1, 0, 0);
    }
    
    const RC_ctrl_t *rc_for_speed = get_remote_control_point();
    int16_t motor7_current = 0;
    if (rc_for_speed != NULL)
    {
        motor7_current = gm6020_control_from_joystick(7, rc_for_speed->rc.ch[1]);
    }
    (void)GM6020_SendCurrentById(7, motor7_current);
    output_currents_5_8[0] = motor5_current;
    output_currents_5_8[1] = motor6_current;
    output_currents_5_8[2] = 0;
    output_currents_5_8[3] = motor8_current;
    CAN_SendMotorCurrents4Ex(&hcan2, MOTOR_STDID_5_8,
                         output_currents_5_8[0], output_currents_5_8[1],
                         output_currents_5_8[2], output_currents_5_8[3]);
    CAN_SendMotorCurrents4Ex(&hcan1, MOTOR_STDID_1_4,
                         output_currents[0], output_currents[1],
                         output_currents[2], output_currents[3]);



    HAL_Delay(CMD_REFRESH_INTERVAL_MS);
  }
    /* USER CODE END WHILE */

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
