/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/*
 * #include "FreeRTOS.h"
	#include "task.h"
	#include "timers.h"
	#include "queue.h"
	#include "semphr.h"
	#include "event_groups.h"
 */
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
#define ESC_MOTOR1_Pin GPIO_PIN_0
#define ESC_MOTOR1_GPIO_Port GPIOA
#define BUZZER_T3CH1_Pin GPIO_PIN_6
#define BUZZER_T3CH1_GPIO_Port GPIOA
#define NRF_CE_Pin GPIO_PIN_12
#define NRF_CE_GPIO_Port GPIOD
#define NRF_CSN_Pin GPIO_PIN_13
#define NRF_CSN_GPIO_Port GPIOD
#define NRF_IRQ_Pin GPIO_PIN_14
#define NRF_IRQ_GPIO_Port GPIOD
#define M8N_TX6_Pin GPIO_PIN_6
#define M8N_TX6_GPIO_Port GPIOC
#define M8N_RX6_Pin GPIO_PIN_7
#define M8N_RX6_GPIO_Port GPIOC
#define ESC_MOTOR2_Pin GPIO_PIN_3
#define ESC_MOTOR2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
