/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "sensor.h"
/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

extern I2C_HandleTypeDef hi2c3;

/* USER CODE BEGIN Private defines */
extern volatile uint8_t i2c_error_event;
extern volatile uint8_t in_sensor_rx_ready;
extern volatile uint8_t ex_sensor_rx_ready;

extern volatile uint8_t in_sensor_count;
extern volatile uint8_t ex_sensor_count;

extern volatile uint32_t i2c_error_code;

extern volatile SENSOR_DATA in_sensor_data;
extern volatile SENSOR_DATA ex_sensor_data;
/* USER CODE END Private defines */

void MX_I2C1_Init(void);
void MX_I2C3_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

