/*
 * HMC5883L.c
 *
 *  Created on: 4 Eyl 2022
 *      Author: FURKAN
 */


#include "HMC5883L.h"
#include "main.h"

#include "sensordriver.h"

extern I2C_HandleTypeDef hi2c2;

uint16_t d;


sensor_status HMC5883L_self_test(void){
	HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout)


}
