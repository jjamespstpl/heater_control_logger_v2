/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define TR1_Pin GPIO_PIN_13
#define TR1_GPIO_Port GPIOC
#define TR2_Pin GPIO_PIN_14
#define TR2_GPIO_Port GPIOC
#define TR3_Pin GPIO_PIN_15
#define TR3_GPIO_Port GPIOC
#define TRIAC1_Pin GPIO_PIN_0
#define TRIAC1_GPIO_Port GPIOF
#define UP_LED_Pin GPIO_PIN_1
#define UP_LED_GPIO_Port GPIOF
#define R_ZCD_Pin GPIO_PIN_2
#define R_ZCD_GPIO_Port GPIOA
#define R_ZCD_EXTI_IRQn EXTI2_3_IRQn
#define Y_ZCD_Pin GPIO_PIN_3
#define Y_ZCD_GPIO_Port GPIOA
#define Y_ZCD_EXTI_IRQn EXTI2_3_IRQn
#define B_ZCD_Pin GPIO_PIN_4
#define B_ZCD_GPIO_Port GPIOA
#define B_ZCD_EXTI_IRQn EXTI4_15_IRQn
#define GSM_TX_Pin GPIO_PIN_5
#define GSM_TX_GPIO_Port GPIOA
#define MCU_RESET_Pin GPIO_PIN_6
#define MCU_RESET_GPIO_Port GPIOA
#define MCU_PWRKEY_Pin GPIO_PIN_7
#define MCU_PWRKEY_GPIO_Port GPIOA
#define GSM_RX_Pin GPIO_PIN_0
#define GSM_RX_GPIO_Port GPIOB
#define CS_TC2_Pin GPIO_PIN_12
#define CS_TC2_GPIO_Port GPIOB
#define R_CS_Pin GPIO_PIN_13
#define R_CS_GPIO_Port GPIOB
#define Y_CS_Pin GPIO_PIN_14
#define Y_CS_GPIO_Port GPIOB
#define B_CS_Pin GPIO_PIN_15
#define B_CS_GPIO_Port GPIOB
#define CS_TC6_Pin GPIO_PIN_8
#define CS_TC6_GPIO_Port GPIOA
#define RTC_INT_Pin GPIO_PIN_6
#define RTC_INT_GPIO_Port GPIOC
#define RTC_INT_EXTI_IRQn EXTI4_15_IRQn
#define LED_1_Pin GPIO_PIN_15
#define LED_1_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_0
#define LED_2_GPIO_Port GPIOD
#define LED_3_Pin GPIO_PIN_1
#define LED_3_GPIO_Port GPIOD
#define SPI1_CS_Pin GPIO_PIN_3
#define SPI1_CS_GPIO_Port GPIOD
#define BTN3_IN_Pin GPIO_PIN_7
#define BTN3_IN_GPIO_Port GPIOB
#define BTN2_IN_Pin GPIO_PIN_8
#define BTN2_IN_GPIO_Port GPIOB
#define BTN1_IN_Pin GPIO_PIN_9
#define BTN1_IN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
