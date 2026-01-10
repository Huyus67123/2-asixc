/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define DIR2_Pin GPIO_PIN_1
#define DIR2_GPIO_Port GPIOA
#define ENA2_Pin GPIO_PIN_2
#define ENA2_GPIO_Port GPIOA
#define StopY_Pin GPIO_PIN_5
#define StopY_GPIO_Port GPIOA
#define StopY_EXTI_IRQn EXTI9_5_IRQn
#define StopX_Pin GPIO_PIN_6
#define StopX_GPIO_Port GPIOA
#define StopX_EXTI_IRQn EXTI9_5_IRQn
#define button1_Pin GPIO_PIN_7
#define button1_GPIO_Port GPIOA
#define button1_EXTI_IRQn EXTI9_5_IRQn
#define button2_Pin GPIO_PIN_1
#define button2_GPIO_Port GPIOB
#define button2_EXTI_IRQn EXTI1_IRQn
#define button3_Pin GPIO_PIN_10
#define button3_GPIO_Port GPIOB
#define button3_EXTI_IRQn EXTI15_10_IRQn
#define button4_Pin GPIO_PIN_11
#define button4_GPIO_Port GPIOB
#define button4_EXTI_IRQn EXTI15_10_IRQn
#define ENA1_Pin GPIO_PIN_14
#define ENA1_GPIO_Port GPIOB
#define DIR1_Pin GPIO_PIN_15
#define DIR1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
