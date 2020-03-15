/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  * Copyright (c) 2018-2019 Thomas Truong
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx.h"
#include "stm32f1xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32utilities.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define COOL_FLOOD_Pin GPIO_PIN_13
#define COOL_FLOOD_GPIO_Port GPIOC
#define COOL_MIST_Pin GPIO_PIN_14
#define COOL_MIST_GPIO_Port GPIOC
#define STEP_ENABLE_Pin GPIO_PIN_15
#define STEP_ENABLE_GPIO_Port GPIOC
#define DIR_X_Pin GPIO_PIN_0
#define DIR_X_GPIO_Port GPIOA
#define STEP_X_Pin GPIO_PIN_1
#define STEP_X_GPIO_Port GPIOA
#define STEP_Z_Pin GPIO_PIN_2
#define STEP_Z_GPIO_Port GPIOA
#define DIR_C_Pin GPIO_PIN_3
#define DIR_C_GPIO_Port GPIOA
#define DIR_Y_Pin GPIO_PIN_4
#define DIR_Y_GPIO_Port GPIOA
#define DIR_Z_Pin GPIO_PIN_5
#define DIR_Z_GPIO_Port GPIOA
#define DIR_A_Pin GPIO_PIN_6
#define DIR_A_GPIO_Port GPIOA
#define DIR_B_Pin GPIO_PIN_7
#define DIR_B_GPIO_Port GPIOA
#define CON_CYCLE_START_Pin GPIO_PIN_0
#define CON_CYCLE_START_GPIO_Port GPIOB
#define CON_CYCLE_START_EXTI_IRQn EXTI0_IRQn
#define CON_FEED_HOLD_Pin GPIO_PIN_1
#define CON_FEED_HOLD_GPIO_Port GPIOB
#define CON_FEED_HOLD_EXTI_IRQn EXTI1_IRQn
#define LIM_X_Pin GPIO_PIN_10
#define LIM_X_GPIO_Port GPIOB
#define LIM_X_EXTI_IRQn EXTI15_10_IRQn
#define LIM_Y_Pin GPIO_PIN_11
#define LIM_Y_GPIO_Port GPIOB
#define LIM_Y_EXTI_IRQn EXTI15_10_IRQn
#define LIM_Z_Pin GPIO_PIN_12
#define LIM_Z_GPIO_Port GPIOB
#define LIM_Z_EXTI_IRQn EXTI15_10_IRQn
#define LIM_A_Pin GPIO_PIN_13
#define LIM_A_GPIO_Port GPIOB
#define LIM_A_EXTI_IRQn EXTI15_10_IRQn
#define LIM_B_Pin GPIO_PIN_14
#define LIM_B_GPIO_Port GPIOB
#define LIM_B_EXTI_IRQn EXTI15_10_IRQn
#define LIM_C_Pin GPIO_PIN_15
#define LIM_C_GPIO_Port GPIOB
#define LIM_C_EXTI_IRQn EXTI15_10_IRQn
#define PWM_SPIN_Pin GPIO_PIN_8
#define PWM_SPIN_GPIO_Port GPIOA
#define STEP_C_Pin GPIO_PIN_9
#define STEP_C_GPIO_Port GPIOA
#define STEP_A_Pin GPIO_PIN_10
#define STEP_A_GPIO_Port GPIOA
#define STEP_B_Pin GPIO_PIN_11
#define STEP_B_GPIO_Port GPIOA
#define STEP_Y_Pin GPIO_PIN_12
#define STEP_Y_GPIO_Port GPIOA
#define PROBE_Pin GPIO_PIN_15
#define PROBE_GPIO_Port GPIOA
#define CON_RESET_Pin GPIO_PIN_3
#define CON_RESET_GPIO_Port GPIOB
#define CON_RESET_EXTI_IRQn EXTI3_IRQn
#define CON_SAFETY_DOOR_Pin GPIO_PIN_4
#define CON_SAFETY_DOOR_GPIO_Port GPIOB
#define CON_SAFETY_DOOR_EXTI_IRQn EXTI4_IRQn
#define AUX_1_Pin GPIO_PIN_5
#define AUX_1_GPIO_Port GPIOB
#define SPIN_DIR_Pin GPIO_PIN_8
#define SPIN_DIR_GPIO_Port GPIOB
#define SPIN_EN_Pin GPIO_PIN_9
#define SPIN_EN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#include "stm32_pin_out.h"

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
