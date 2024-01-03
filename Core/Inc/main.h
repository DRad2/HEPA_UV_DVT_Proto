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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define _24V_SCLD_Pin GPIO_PIN_0
#define _24V_SCLD_GPIO_Port GPIOC
#define _3V3_SCLD_Pin GPIO_PIN_1
#define _3V3_SCLD_GPIO_Port GPIOC
#define UV_NO_MCU_Pin GPIO_PIN_2
#define UV_NO_MCU_GPIO_Port GPIOC
#define UV_NO_MCU_EXTI_IRQn EXTI2_IRQn
#define LED_DRIVE_Pin GPIO_PIN_1
#define LED_DRIVE_GPIO_Port GPIOA
#define UV_ON_OFF_MCU_Pin GPIO_PIN_4
#define UV_ON_OFF_MCU_GPIO_Port GPIOA
#define HEPA_FG_Pin GPIO_PIN_5
#define HEPA_FG_GPIO_Port GPIOA
#define HEPA_PWM_Pin GPIO_PIN_6
#define HEPA_PWM_GPIO_Port GPIOA
#define HEPA_ON_OFF_Pin GPIO_PIN_7
#define HEPA_ON_OFF_GPIO_Port GPIOA
#define EEPROM_SCL_Pin GPIO_PIN_4
#define EEPROM_SCL_GPIO_Port GPIOC
#define UV_B_CTRL_Pin GPIO_PIN_5
#define UV_B_CTRL_GPIO_Port GPIOC
#define UV_G_CTRL_Pin GPIO_PIN_0
#define UV_G_CTRL_GPIO_Port GPIOB
#define UV_R_CTRL_Pin GPIO_PIN_1
#define UV_R_CTRL_GPIO_Port GPIOB
#define UV_W_CTRL_Pin GPIO_PIN_2
#define UV_W_CTRL_GPIO_Port GPIOB
#define HEPA_NO_MCU_Pin GPIO_PIN_10
#define HEPA_NO_MCU_GPIO_Port GPIOB
#define HEPA_NO_MCU_EXTI_IRQn EXTI15_10_IRQn
#define HEPA_B_CTRL_Pin GPIO_PIN_11
#define HEPA_B_CTRL_GPIO_Port GPIOB
#define DOOR_OPEN_MCU_Pin GPIO_PIN_7
#define DOOR_OPEN_MCU_GPIO_Port GPIOC
#define DOOR_OPEN_MCU_EXTI_IRQn EXTI9_5_IRQn
#define EEPROM_SDA_Pin GPIO_PIN_8
#define EEPROM_SDA_GPIO_Port GPIOA
#define HEPA_R_CTRL_Pin GPIO_PIN_9
#define HEPA_R_CTRL_GPIO_Port GPIOA
#define HEPA_W_CTRL_Pin GPIO_PIN_10
#define HEPA_W_CTRL_GPIO_Port GPIOA
#define nEEPROM_WP_Pin GPIO_PIN_10
#define nEEPROM_WP_GPIO_Port GPIOC
#define POS_SW_MCU_Pin GPIO_PIN_11
#define POS_SW_MCU_GPIO_Port GPIOC
#define POS_SW_MCU_EXTI_IRQn EXTI15_10_IRQn
#define HEPA_B_CTRLB4_Pin GPIO_PIN_4
#define HEPA_B_CTRLB4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
