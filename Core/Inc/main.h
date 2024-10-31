/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../RkSd/common.h"
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
#define LED13_Pin GPIO_PIN_13
#define LED13_GPIO_Port GPIOC
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define A2_Pin GPIO_PIN_1
#define A2_GPIO_Port GPIOA
#define A3_Pin GPIO_PIN_2
#define A3_GPIO_Port GPIOA
#define A4_Pin GPIO_PIN_3
#define A4_GPIO_Port GPIOA
#define A5_Pin GPIO_PIN_4
#define A5_GPIO_Port GPIOA
#define A0_Pin GPIO_PIN_7
#define A0_GPIO_Port GPIOA
#define A1_Pin GPIO_PIN_0
#define A1_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_1
#define D2_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_2
#define D3_GPIO_Port GPIOB
#define nCS_Pin GPIO_PIN_10
#define nCS_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_12
#define D4_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_13
#define D5_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_14
#define D6_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_15
#define D7_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_8
#define SD_CS_GPIO_Port GPIOA
#define RXRDY_Pin GPIO_PIN_9
#define RXRDY_GPIO_Port GPIOA
#define TXRDY_Pin GPIO_PIN_10
#define TXRDY_GPIO_Port GPIOA
#define A6_Pin GPIO_PIN_6
#define A6_GPIO_Port GPIOB
#define A7_Pin GPIO_PIN_7
#define A7_GPIO_Port GPIOB
#define D0_Pin GPIO_PIN_8
#define D0_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_9
#define D1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void RkSd_main();
static inline void DATA_BUS_IN ()
{
#if 0
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  GPIO_InitStruct.Pin = D0_Pin | D1_Pin | D2_Pin | D3_Pin | D4_Pin | D5_Pin | D6_Pin | D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);
#else
  uint32_t m = 0b11111111000011110000000000111100;
  GPIOB->MODER = (GPIOB->MODER & ~m);
#endif
}

static inline void DATA_BUS_OUT ()
{
#if 0
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  GPIO_InitStruct.Pin = D0_Pin | D1_Pin | D2_Pin | D3_Pin | D4_Pin | D5_Pin  | D6_Pin | D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);
#else
  uint32_t v = 0b01010101000001010000000000010100;
  uint32_t m = 0b11111111000011110000000000111100;
  GPIOB->MODER = (GPIOB->MODER & ~m) | v;
#endif
}

static inline void WRITE_DATA(uint8_t val)
{
  uint32_t portVal = ((val & 0b11110011) << 8) | ((val & 0b1100) >> 1);
  GPIOB->ODR = portVal;
}

static inline uint8_t READ_DATA()
{
  uint32_t idr = GPIOB->IDR;
  return ((idr >> 8) & 0b11110011) | ((idr & 0b110) << 1);
}

static inline uint32_t READ_ADDR()
{
  uint32_t pa = GPIOA->IDR;
  uint32_t pb = GPIOB->IDR;
  //               D7,D6                D1                D0                        D5-D2
  //               B7,B6                B0                A7                        A4-A1
  uint32_t addr = (pb & 0b11000000) | ((pb & 1) << 1) | ((pa & 0b10000000) >> 7) | ((pa & 0b11110) << 1);
  return addr;
}


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
