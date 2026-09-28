/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define P_SGNL1_Pin GPIO_PIN_0
#define P_SGNL1_GPIO_Port GPIOA
#define P_SGNL2_Pin GPIO_PIN_1
#define P_SGNL2_GPIO_Port GPIOA
#define ABS_SGNL2_2_Pin GPIO_PIN_2
#define ABS_SGNL2_2_GPIO_Port GPIOA
#define ABS_SGNL1_1_Pin GPIO_PIN_3
#define ABS_SGNL1_1_GPIO_Port GPIOA
#define ABS_SGNL2_1_Pin GPIO_PIN_5
#define ABS_SGNL2_1_GPIO_Port GPIOA
#define ABS_SGNL1_2_Pin GPIO_PIN_2
#define ABS_SGNL1_2_GPIO_Port GPIOB
#define MCU_TXD_Pin GPIO_PIN_10
#define MCU_TXD_GPIO_Port GPIOB
#define MCU_RXD_Pin GPIO_PIN_11
#define MCU_RXD_GPIO_Port GPIOB
#define MCU_PWRK_Pin GPIO_PIN_6
#define MCU_PWRK_GPIO_Port GPIOC
#define MCU_RST_Pin GPIO_PIN_7
#define MCU_RST_GPIO_Port GPIOC
#define STMUSB_D_N_Pin GPIO_PIN_11
#define STMUSB_D_N_GPIO_Port GPIOA
#define STMUSB_D_P_Pin GPIO_PIN_12
#define STMUSB_D_P_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define CAN1RX_Pin GPIO_PIN_3
#define CAN1RX_GPIO_Port GPIOB
#define CAN1TX_Pin GPIO_PIN_4
#define CAN1TX_GPIO_Port GPIOB
#define CAN2RX_Pin GPIO_PIN_5
#define CAN2RX_GPIO_Port GPIOB
#define CAN2TX_Pin GPIO_PIN_6
#define CAN2TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
