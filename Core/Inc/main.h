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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
	RAIN_DETECTED,
	RAIN_STOP
}SYSTEM_STATUS;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern volatile SYSTEM_STATUS system_status;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOA
#define RAIN_DETECTION_Pin GPIO_PIN_1
#define RAIN_DETECTION_GPIO_Port GPIOA
#define RAIN_DETECTION_EXTI_IRQn EXTI1_IRQn
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define MOTOR_BR_2_Pin GPIO_PIN_3
#define MOTOR_BR_2_GPIO_Port GPIOA
#define MOTOR_FR_1_Pin GPIO_PIN_5
#define MOTOR_FR_1_GPIO_Port GPIOA
#define EX_SENSOR_SCL_Pin GPIO_PIN_7
#define EX_SENSOR_SCL_GPIO_Port GPIOA
#define MOTOR_FR_2_Pin GPIO_PIN_1
#define MOTOR_FR_2_GPIO_Port GPIOB
#define IN_SENSOR_SCL_Pin GPIO_PIN_9
#define IN_SENSOR_SCL_GPIO_Port GPIOA
#define IN_SENSOR_SDA_Pin GPIO_PIN_10
#define IN_SENSOR_SDA_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
#define EX_SENSOR_SDA_Pin GPIO_PIN_4
#define EX_SENSOR_SDA_GPIO_Port GPIOB
#define MOTOR_BR_1_Pin GPIO_PIN_6
#define MOTOR_BR_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
