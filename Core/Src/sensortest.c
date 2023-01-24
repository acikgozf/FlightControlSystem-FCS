/*
 * sensortest.c
 *
 *  Created on: Dec 21, 2021
 *      Author: C.TASDEMIR
 */



#include "mpu6050.h"
//#include "usbd_cdc_if.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"

static SensorData_t sensorData;
extern UART_HandleTypeDef huart5;


void sensorTest_init(void)
{
    MPU6050_initialize(&sensorData, FS_1000, AFS_4G);
}

void sensorTest_print_acc_values(void)
{
    char buffer[100];
    int32_t len;

    MPU6050_read_data(&sensorData);

    len = sprintf(buffer,"Acc: X:%f Y:%f Z:%f\r\n",sensorData.acc.X, sensorData.acc.Y, sensorData.acc.Z );
   //CDC_Transmit_FS((uint8_t *)buffer, len);
	HAL_UART_Transmit(&huart5,buffer , strlen(buffer), HAL_MAX_DELAY);
}

void sensorTest_print_gyro_values(void)
{
    char buffer[100];
    int32_t len;

    MPU6050_read_data(&sensorData);

    len = sprintf(buffer,"Gyro: X:%f Y:%f Z:%f\r\n",sensorData.gyro.X, sensorData.gyro.Y, sensorData.gyro.Z );
    //CDC_Transmit_FS((uint8_t *)buffer, len);
	HAL_UART_Transmit(&huart5, buffer, strlen(buffer), HAL_MAX_DELAY);
}
