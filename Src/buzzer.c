/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    buzzer.c
  * @brief   This file provides code for buzzer control functionality
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
#include "buzzer.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* USER CODE BEGIN Private variables */

static bool buzzer_initialized = false;

/* USER CODE END Private variables */

/* USER CODE BEGIN Private function prototypes */

/**
 * @brief Calculate PWM parameters
 * @param frequency Target frequency
 * @param prescaler Prescaler pointer
 * @param period Period value pointer
 * @retval None
 */
static void Buzzer_CalculatePWM(uint32_t frequency, uint32_t *prescaler, uint32_t *period);

/* USER CODE END Private function prototypes */

/* USER CODE BEGIN Private user code */

/**
 * @brief Calculate PWM parameters
 * @param frequency Target frequency
 * @param prescaler Prescaler pointer
 * @param period Period value pointer
 * @retval None
 */
static void Buzzer_CalculatePWM(uint32_t frequency, uint32_t *prescaler, uint32_t *period)
{
    // PWM frequency = System clock frequency / ((Prescaler + 1) * (Period + 1))
    // For better accuracy, we select an appropriate prescaler value
    
    uint32_t target_period = SYSTEM_CLOCK_FREQ / frequency;
    
    // If target period is too large, need to increase prescaler
    if (target_period > 65535) {
        *prescaler = (target_period / 65535) - 1;
        *period = target_period / (*prescaler + 1) - 1;
    } else {
        *prescaler = 0;
        *period = target_period - 1;
    }
    
    // Ensure values are within valid range
    if (*prescaler > 65535) {
        *prescaler = 65535;
    }
    if (*period > 65535) {
        *period = 65535;
    }
}

/* USER CODE END Private user code */

/* USER CODE BEGIN 0 */

/**
 * @brief Initialize buzzer
 * @retval None
 */
void Buzzer_Init(void)
{
    if (!buzzer_initialized) {
        // Start TIM4 PWM channel 3
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
        buzzer_initialized = true;
    }
}

/**
 * @brief Start buzzer
 * @param frequency Buzzer frequency (Hz)
 * @retval None
 */
void Buzzer_Start(uint32_t frequency)
{
    if (!buzzer_initialized) {
        Buzzer_Init();
    }
    
    uint32_t prescaler, period;
    Buzzer_CalculatePWM(frequency, &prescaler, &period);
    
    // Set prescaler and period
    __HAL_TIM_SET_PRESCALER(&htim4, prescaler);
    __HAL_TIM_SET_AUTORELOAD(&htim4, period);
    
    // Set duty cycle to 50%
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, period / 2);
}

/**
 * @brief Stop buzzer
 * @retval None
 */
void Buzzer_Stop(void)
{
    if (buzzer_initialized) {
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
    }
}

/**
 * @brief Beep for specified duration
 * @param frequency Buzzer frequency (Hz)
 * @param duration_ms Duration in milliseconds
 * @retval None
 */
void Buzzer_Beep(uint32_t frequency, uint32_t duration_ms)
{
    Buzzer_Start(frequency);
    HAL_Delay(duration_ms);
    Buzzer_Stop();
}

/**
 * @brief Boot beep once
 * @retval None
 */
void Buzzer_BootBeep(void)
{
    Buzzer_Beep(BUZZER_DEFAULT_FREQ, BUZZER_BOOT_BEEP);
}

/* USER CODE END 0 */
