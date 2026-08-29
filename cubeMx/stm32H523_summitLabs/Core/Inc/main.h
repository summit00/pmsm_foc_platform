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
#include "stm32h5xx_hal.h"

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
#define DCBus_Pin GPIO_PIN_3
#define DCBus_GPIO_Port GPIOC
#define Iu_Pin GPIO_PIN_0
#define Iu_GPIO_Port GPIOA
#define Iv_Pin GPIO_PIN_1
#define Iv_GPIO_Port GPIOA
#define Iw_Pin GPIO_PIN_2
#define Iw_GPIO_Port GPIOA
#define DRV_Enable_Pin GPIO_PIN_10
#define DRV_Enable_GPIO_Port GPIOB
#define DRV_Fault_Pin GPIO_PIN_12
#define DRV_Fault_GPIO_Port GPIOB
#define USB_Vbus_Pin GPIO_PIN_10
#define USB_Vbus_GPIO_Port GPIOA
#define LED_Status_Pin GPIO_PIN_3
#define LED_Status_GPIO_Port GPIOD
#define LED_Error_Pin GPIO_PIN_5
#define LED_Error_GPIO_Port GPIOD
#define ChipSelect_Pin GPIO_PIN_6
#define ChipSelect_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
