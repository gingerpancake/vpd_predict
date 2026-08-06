/*
 * sensor.c
 *
 *  Created on: 2026. 8. 4.
 *      Author: karma
 */

#include "sensor.h"
#include "i2c.h"

/* values */
uint8_t icmd[2] = {0x24, 0x00};
uint8_t ecmd[2] = {0x24, 0x00};

uint8_t ircv[6];
uint8_t ercv[6];
/* values */

volatile SensorState sensor_state = SENSOR_STATE_IDLE;

/* functions */
HAL_StatusTypeDef In_Sensor_Read(void)
{
    HAL_StatusTypeDef status;

    if (sensor_state != SENSOR_STATE_IDLE)
    {
        return HAL_BUSY;
    }

    sensor_state = SENSOR_STATE_IN_WAIT;

    status = HAL_I2C_Master_Transmit_IT(&hi2c1, IN_SENSOR_ADDR, icmd, sizeof(icmd));

    if (status != HAL_OK)
    {
        sensor_state = SENSOR_STATE_IDLE;
    }

    return status;
}


HAL_StatusTypeDef Ex_Sensor_Read(void)
{
    HAL_StatusTypeDef status;

    if (sensor_state != SENSOR_STATE_IDLE)
    {
        return HAL_BUSY;
    }

    sensor_state = SENSOR_STATE_EX_WAIT;

    status = HAL_I2C_Master_Transmit_IT(&hi2c1, EX_SENSOR_ADDR, ecmd, sizeof(ecmd));

    if (status != HAL_OK)
    {
        sensor_state = SENSOR_STATE_IDLE;
    }

    return status;
}

/* functions */
