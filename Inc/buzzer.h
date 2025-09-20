/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    buzzer.h
  * @brief   This file contains all the function prototypes for
  *          the buzzer.c file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUZZER_H__
#define __BUZZER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

// Buzzer parameter definitions
#define BUZZER_FREQ_1KHZ     1000    // 1kHz frequency
#define BUZZER_FREQ_2KHZ     2000    // 2kHz frequency
#define BUZZER_FREQ_3KHZ     3000    // 3kHz frequency
#define BUZZER_DEFAULT_FREQ  BUZZER_FREQ_1KHZ

// Buzzer duration definitions (milliseconds)
#define BUZZER_SHORT_BEEP    100     // Short beep
#define BUZZER_MEDIUM_BEEP   200     // Medium beep
#define BUZZER_LONG_BEEP     500     // Long beep
#define BUZZER_BOOT_BEEP     BUZZER_MEDIUM_BEEP  // Boot beep

// System clock frequency (calculated from SystemClock_Config: 72MHz)
#define SYSTEM_CLOCK_FREQ    72000000

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/**
 * @brief Initialize buzzer
 * @retval None
 */
void Buzzer_Init(void);

/**
 * @brief Start buzzer
 * @param frequency Buzzer frequency (Hz)
 * @retval None
 */
void Buzzer_Start(uint32_t frequency);

/**
 * @brief Stop buzzer
 * @retval None
 */
void Buzzer_Stop(void);

/**
 * @brief Beep for specified duration
 * @param frequency Buzzer frequency (Hz)
 * @param duration_ms Duration in milliseconds
 * @retval None
 */
void Buzzer_Beep(uint32_t frequency, uint32_t duration_ms);

/**
 * @brief Boot beep once
 * @retval None
 */
void Buzzer_BootBeep(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __BUZZER_H__ */
