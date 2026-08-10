/*
 * sensor.h
 *
 *  Created on: 2026. 8. 4.
 *      Author: karma
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include "main.h"

/* i2c address value */
#define SENSOR_ADDR	(0x44 << 1)
/* i2c address value */

extern uint8_t icmd[2];
extern uint8_t ecmd[2];

extern uint8_t ircv[6];
extern uint8_t ercv[6];

/* typedef */
typedef struct {
	uint8_t temp_msb;
	uint8_t temp_lsb;
	uint8_t temp_crc;
	uint8_t humi_msb;
	uint8_t humi_lsb;
	uint8_t humi_crc;
}SENSOR_DATA;

typedef enum
{
    SENSOR_STATE_IDLE = 0,
    SENSOR_STATE_IN_WAIT,
    SENSOR_STATE_EX_WAIT
} SensorState;

extern volatile SensorState sensor_state;
/* typedef */

/* function */
HAL_StatusTypeDef In_Sensor_Read(void);
HAL_StatusTypeDef Ex_Sensor_Read(void);
/* function */


#endif /* INC_SENSOR_H_ */
