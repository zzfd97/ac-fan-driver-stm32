/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#define zero_crossing_detection_Pin GPIO_PIN_0
#define zero_crossing_detection_GPIO_Port GPIOC
#define zero_crossing_detection_EXTI_IRQn EXTI0_IRQn
#define gate_1_Pin GPIO_PIN_1
#define gate_1_GPIO_Port GPIOC
#define gate_2_Pin GPIO_PIN_2
#define gate_2_GPIO_Port GPIOC
#define gate_3_Pin GPIO_PIN_3
#define gate_3_GPIO_Port GPIOC
#define ntc_1_Pin GPIO_PIN_0
#define ntc_1_GPIO_Port GPIOA
#define ntc_2_Pin GPIO_PIN_1
#define ntc_2_GPIO_Port GPIOA
#define ntc_3_Pin GPIO_PIN_2
#define ntc_3_GPIO_Port GPIOA
#define ntc_4_Pin GPIO_PIN_3
#define ntc_4_GPIO_Port GPIOA
#define ntc_5_Pin GPIO_PIN_4
#define ntc_5_GPIO_Port GPIOA
#define ntc_6_Pin GPIO_PIN_5
#define ntc_6_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_12
#define LED_G_GPIO_Port GPIOD
#define LED_R_Pin GPIO_PIN_14
#define LED_R_GPIO_Port GPIOD
#define rs_dir_Pin GPIO_PIN_5
#define rs_dir_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
