/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SPI_SEL2_Pin|SPI_SEL3_Pin|I2C_OE_Pin|SPI_SEL0_Pin 
                          |SPI_SEL1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_X_Pin|DIR_Y_Pin|DIR_Z_Pin|DIR_A_Pin 
                          |DIR_B_Pin|DIR_C_Pin|STEP_ENABLE_Pin|STEP_X_Pin 
                          |STEP_Y_Pin|STEP_Z_Pin|STEP_A_Pin|STEP_B_Pin 
                          |STEP_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, AUX_5_Pin|AUX_6_Pin|AUX_7_Pin|AUX_8_Pin 
                          |AUX_1_Pin|AUX_2_Pin|AUX_3_Pin|AUX_4_Pin 
                          |SPIN_DIR_Pin|SPIN_EN_Pin|COOL_MIST_Pin|COOL_FLOOD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PEPin PEPin PEPin PEPin */
  GPIO_InitStruct.Pin = SPI_SEL2_Pin|SPI_SEL3_Pin|SPI_SEL0_Pin|SPI_SEL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = I2C_OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I2C_OE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin PAPin 
                           PAPin PAPin PAPin PAPin 
                           PAPin PAPin PAPin PAPin 
                           PAPin */
  GPIO_InitStruct.Pin = DIR_X_Pin|DIR_Y_Pin|DIR_Z_Pin|DIR_A_Pin 
                          |DIR_B_Pin|DIR_C_Pin|STEP_ENABLE_Pin|STEP_X_Pin 
                          |STEP_Y_Pin|STEP_Z_Pin|STEP_A_Pin|STEP_B_Pin 
                          |STEP_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin 
                           PBPin */
  GPIO_InitStruct.Pin = CON_FEED_HOLD_Pin|CON_CYCLE_START_Pin|CON_RESET_Pin|INT_LIMITS_Pin 
                          |CON_SAFETY_DOOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PDPin PDPin PDPin PDPin 
                           PDPin PDPin PDPin PDPin 
                           PDPin PDPin PDPin PDPin */
  GPIO_InitStruct.Pin = AUX_5_Pin|AUX_6_Pin|AUX_7_Pin|AUX_8_Pin 
                          |AUX_1_Pin|AUX_2_Pin|AUX_3_Pin|AUX_4_Pin 
                          |SPIN_DIR_Pin|SPIN_EN_Pin|COOL_MIST_Pin|COOL_FLOOD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = PROBE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PROBE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
