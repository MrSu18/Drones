/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "uart.h"
#include "SPL06_001.h"
#include "NRF24L01.h"
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED3_Pin GPIO_PIN_1
#define LED3_GPIO_Port GPIOA
#define NRF24L01_CSN_Pin GPIO_PIN_4
#define NRF24L01_CSN_GPIO_Port GPIOA
#define NRF24L01_CE_Pin GPIO_PIN_2
#define NRF24L01_CE_GPIO_Port GPIOB
#define NRF24L01_IRQ_Pin GPIO_PIN_10
#define NRF24L01_IRQ_GPIO_Port GPIOB
#define NRF24L01_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define LED4_Pin GPIO_PIN_8
#define LED4_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
//#define NRF24L01_IRQ_EXTI_IRQn EXTI15_10_IRQn  //EXTI9_5_IRQn
#define LED0_Pin GPIO_PIN_13
#define LED0_GPIO_Port GPIOC

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
