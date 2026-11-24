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
#include "stm32h7xx_hal.h"

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
#define UBC_P_Pin GPIO_PIN_9
#define UBC_P_GPIO_Port GPIOF
#define UBC_N_Pin GPIO_PIN_10
#define UBC_N_GPIO_Port GPIOF
#define UDC_Pin GPIO_PIN_0
#define UDC_GPIO_Port GPIOC
#define PFC_L_PWM_N_Pin GPIO_PIN_5
#define PFC_L_PWM_N_GPIO_Port GPIOA
#define IIN_P_Pin GPIO_PIN_6
#define IIN_P_GPIO_Port GPIOA
#define IIN_N_Pin GPIO_PIN_7
#define IIN_N_GPIO_Port GPIOA
#define UAB_P_Pin GPIO_PIN_4
#define UAB_P_GPIO_Port GPIOC
#define UAB_N_Pin GPIO_PIN_5
#define UAB_N_GPIO_Port GPIOC
#define IO_P_Pin GPIO_PIN_0
#define IO_P_GPIO_Port GPIOB
#define IO_N_Pin GPIO_PIN_1
#define IO_N_GPIO_Port GPIOB
#define UIN_P_Pin GPIO_PIN_11
#define UIN_P_GPIO_Port GPIOF
#define UIN_N_Pin GPIO_PIN_12
#define UIN_N_GPIO_Port GPIOF
#define INV_U_PWM_N_Pin GPIO_PIN_8
#define INV_U_PWM_N_GPIO_Port GPIOE
#define INV_U_PWM_Pin GPIO_PIN_9
#define INV_U_PWM_GPIO_Port GPIOE
#define INV_V_PWM_N_Pin GPIO_PIN_10
#define INV_V_PWM_N_GPIO_Port GPIOE
#define INV_V_PWM_Pin GPIO_PIN_11
#define INV_V_PWM_GPIO_Port GPIOE
#define INV_W_PWM_N_Pin GPIO_PIN_12
#define INV_W_PWM_N_GPIO_Port GPIOE
#define INV_W_PWM_Pin GPIO_PIN_13
#define INV_W_PWM_GPIO_Port GPIOE
#define DRIVER_DIS_HIGH_Pin GPIO_PIN_14
#define DRIVER_DIS_HIGH_GPIO_Port GPIOE
#define PFC_N_PWM_N_Pin GPIO_PIN_14
#define PFC_N_PWM_N_GPIO_Port GPIOB
#define EXTERNAL_FAULT_HIGH_Pin GPIO_PIN_2
#define EXTERNAL_FAULT_HIGH_GPIO_Port GPIOG
#define EXTERNAL_FAULT_HIGH_EXTI_IRQn EXTI2_IRQn
#define PFC_L_PWM_Pin GPIO_PIN_6
#define PFC_L_PWM_GPIO_Port GPIOC
#define PFC_N_PWM_Pin GPIO_PIN_7
#define PFC_N_PWM_GPIO_Port GPIOC
#define KEY_START_N_Pin GPIO_PIN_0
#define KEY_START_N_GPIO_Port GPIOD
#define KEY_STOP_N_Pin GPIO_PIN_1
#define KEY_STOP_N_GPIO_Port GPIOD
#define KEY_FREQ_N_Pin GPIO_PIN_2
#define KEY_FREQ_N_GPIO_Port GPIOD
#define KEY_CLEAR_N_Pin GPIO_PIN_3
#define KEY_CLEAR_N_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
