/*
 * RF.h
 *
 *  Created on: 5 Oca 2023
 *      Author: FURKAN
 */

#ifndef SENSOR_DRIVERS_INC_RF_H_
#define SENSOR_DRIVERS_INC_RF_H_

#include "stdint.h"
#include "stm32f4xx_hal.h"


typedef struct{

	SPI_HandleTypeDef *hspi1;



}rf;


typedef struct{

	uint8_t rx_buf;
	uint8_t tx_buf;

}rfData;

#endif /* SENSOR_DRIVERS_INC_RF_H_ */
