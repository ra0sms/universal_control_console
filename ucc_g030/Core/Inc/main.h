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
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_usart.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_dma.h"

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
#define BTN_470_Pin LL_GPIO_PIN_9
#define BTN_470_GPIO_Port GPIOB
#define BTN_390_Pin LL_GPIO_PIN_14
#define BTN_390_GPIO_Port GPIOC
#define BTN_430_Pin LL_GPIO_PIN_15
#define BTN_430_GPIO_Port GPIOC
#define OE_Pin LL_GPIO_PIN_0
#define OE_GPIO_Port GPIOA
#define RCLK_Pin LL_GPIO_PIN_3
#define RCLK_GPIO_Port GPIOA
#define BUZZER_Pin LL_GPIO_PIN_4
#define BUZZER_GPIO_Port GPIOA
#define BTN_510_Pin LL_GPIO_PIN_5
#define BTN_510_GPIO_Port GPIOA
#define BTN_560_Pin LL_GPIO_PIN_6
#define BTN_560_GPIO_Port GPIOA
#define PTT_IN_Pin LL_GPIO_PIN_7
#define PTT_IN_GPIO_Port GPIOA
#define BTN_N_Pin LL_GPIO_PIN_0
#define BTN_N_GPIO_Port GPIOB
#define BTN_NE_Pin LL_GPIO_PIN_1
#define BTN_NE_GPIO_Port GPIOB
#define BTN_E_Pin LL_GPIO_PIN_2
#define BTN_E_GPIO_Port GPIOB
#define TX_GND_Pin LL_GPIO_PIN_8
#define TX_GND_GPIO_Port GPIOA
#define BTN_300_Pin LL_GPIO_PIN_6
#define BTN_300_GPIO_Port GPIOC
#define PA_ON_Pin LL_GPIO_PIN_15
#define PA_ON_GPIO_Port GPIOA
#define BTN_SE_Pin LL_GPIO_PIN_3
#define BTN_SE_GPIO_Port GPIOB
#define BTN_S_Pin LL_GPIO_PIN_4
#define BTN_S_GPIO_Port GPIOB
#define BTN_SW_Pin LL_GPIO_PIN_5
#define BTN_SW_GPIO_Port GPIOB
#define BTN_W_Pin LL_GPIO_PIN_6
#define BTN_W_GPIO_Port GPIOB
#define BTN_NW_Pin LL_GPIO_PIN_7
#define BTN_NW_GPIO_Port GPIOB
#define BTN_PA_Pin LL_GPIO_PIN_8
#define BTN_PA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
