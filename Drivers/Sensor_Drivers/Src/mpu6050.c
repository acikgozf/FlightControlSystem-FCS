/*
 * mpu6050.c
 *
 *  Created on: 4 Eyl 2022
 *      Author: FURKAN
 */


#include "mpu6050.h"
#include "main.h"
#include "sensordriver.h"
#include "mpu6050.h"
//#include "usbd_cdc_if.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "math.h"
#include "usbd_cdc_if.h"

#define Rad_to_deg  			57.3248408
#define gravity_constant_value	9.810000000
#define ALPHA 					0.05f
#define SAMPLE_TIME_USB 		20


static imu_datalar data;
static SensorData_t sensorData;
extern UART_HandleTypeDef huart5;

static sensor_status_e MPU6050_set_acc_range(SensorData_t *pSensor, afs_sel_e accRange);
static sensor_status_e MPU6050_set_gyro_range(SensorData_t *pSensor, fs_sel_e gyroRange);



void mpu6050_init(void){

	MPU6050_initialize(&sensorData, FS_1000, AFS_4G);


}

sensor_status_e MPU6050_initialize( SensorData_t *pSensor, fs_sel_e gyroConfig, afs_sel_e acc_config )
{
    sensor_status_e retVal;

    retVal = MPU6050_set_gyro_range( pSensor, gyroConfig );

    if (retVal == SENSOR_OK) {
        retVal = MPU6050_set_acc_range( pSensor, acc_config );

        if (retVal == SENSOR_OK) {
            retVal = MPU6050_set_sleep_mode( SLEEPMODE_OFF );
        }
    }


    return retVal;
}

sensor_status_e MPU6050_set_gyro_range(SensorData_t *pSensor, fs_sel_e gyroRange)
{
	sensor_status_e retVal;
	uint8_t configReg = sensor_read_register8(MPU6050_I2C_ADRESS_AD0, MPU_REG_GYRO_CONFIG );

	configReg |= ((uint32_t)gyroRange << MPU_REG_GYRO_CONFIG_GYRO_RANGE_BITS_POSITION );

	retVal = sensor_write_register8(MPU6050_I2C_ADRESS_AD0, MPU_REG_GYRO_CONFIG, configReg);

	switch (gyroRange)
	{
	 case FS_250:  pSensor->gyro_co = 131.0; break;
	 case FS_500:  pSensor->gyro_co = 65.5; break;
	 case FS_1000: pSensor->gyro_co = 32.8; break;
	 case FS_2000: pSensor->gyro_co = 16.4; break;
	 default: retVal = SENSOR_ERROR; break;
	}

	return retVal;
}

sensor_status_e MPU6050_set_acc_range(SensorData_t *pSensor, afs_sel_e accRange)
{
	sensor_status_e retVal;
	uint8_t configReg = sensor_read_register8(MPU6050_I2C_ADRESS_AD0, MPU_REG_ACCEL_CONFIG );

	configReg |= ( (uint32_t) accRange << MPU_REG_ACC_CONFIG_ACC_RANGE_BITS_POSITION );

	retVal = sensor_write_register8(MPU6050_I2C_ADRESS_AD0, MPU_REG_ACCEL_CONFIG, configReg);

	switch (accRange)
	{
	 case AFS_2G:  pSensor->acc_co = 16384; break;
	 case AFS_4G:  pSensor->acc_co = 8192; break;
	 case AFS_8G:  pSensor->acc_co = 4096; break;
	 case AFS_16G: pSensor->acc_co = 2048; break;
	 default: retVal = SENSOR_ERROR; break;
	}

	return retVal;
}

sensor_status_e MPU6050_test_sensor()
{
	sensor_status_e retVal;
	retVal = sensor_test_device(MPU6050_I2C_ADRESS_AD0);
	return retVal;
}


uint8_t MPU6050_read_id(void)
{
	uint8_t id = 0;
	id = sensor_read_register8(MPU6050_I2C_ADRESS_AD0,MPU_REG_WHO_AM_I );
	return id;
}

sensor_status_e MPU6050_set_sleep_mode(sleepmode_e sleepmode)
{
	sensor_status_e retVal;
	uint8_t powerReg = sensor_read_register8(MPU6050_I2C_ADRESS_AD0, MPU_REG_PWR_MGMT_1 );

	if(SLEEPMODE_ON == sleepmode) {
		SET_BIT(powerReg,1<<MPU_BIT_PWR_MGMT_1_SLEEP_MODE);
	}
	else {
		CLEAR_BIT(powerReg,1<<MPU_BIT_PWR_MGMT_1_SLEEP_MODE);
	}
	retVal = sensor_write_register8(MPU6050_I2C_ADRESS_AD0, MPU_REG_PWR_MGMT_1, powerReg);

	powerReg = sensor_read_register8(MPU6050_I2C_ADRESS_AD0, MPU_REG_PWR_MGMT_1 );

	return retVal;
}


sensor_status_e MPU6050_read_data(SensorData_t *pSensorData)
{
	sensor_status_e retVal;
	uint8_t buffer[14];
	retVal =  sensor_read_bytes(MPU6050_I2C_ADRESS_AD0, MPU_REG_ACCEL_XOUT_H, buffer, 14);

    if (retVal == SENSOR_OK) {
        pSensorData->accRaw.X = (int16_t) ((buffer[0] << 8) | buffer[1]);
        pSensorData->accRaw.Y = (int16_t) ((buffer[2] << 8) | buffer[3]);
        pSensorData->accRaw.Z = (int16_t) ((buffer[4] << 8) | buffer[5]);

        pSensorData->gyroRaw.X = (int16_t) ((buffer[8] << 8) | buffer[9]);
        pSensorData->gyroRaw.Y = (int16_t) ((buffer[10] << 8) | buffer[11]);
        pSensorData->gyroRaw.Z = (int16_t) ((buffer[12] << 8) | buffer[13]);

        pSensorData->acc.X = pSensorData->accRaw.X / pSensorData->acc_co;
        pSensorData->acc.Y = pSensorData->accRaw.Y / pSensorData->acc_co;
        pSensorData->acc.Z = pSensorData->accRaw.Z / pSensorData->acc_co;

        pSensorData->gyro.X = pSensorData->gyroRaw.X / pSensorData->gyro_co;
        pSensorData->gyro.Y = pSensorData->gyroRaw.Y / pSensorData->gyro_co;
        pSensorData->gyro.Z = pSensorData->gyroRaw.Z / pSensorData->gyro_co;

        return retVal;
    } else {
        return SENSOR_ERROR;
    }
}



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

void sensorTest_print_gyro_values(void){
    char buffer[100];
    int32_t len;

    MPU6050_read_data(&sensorData);

    len = sprintf(buffer,"Gyro: X:%f Y:%f Z:%f\r\n",sensorData.gyro.X, sensorData.gyro.Y, sensorData.gyro.Z );
    //CDC_Transmit_FS((uint8_t *)buffer, len);
	HAL_UART_Transmit(&huart5, buffer, strlen(buffer), HAL_MAX_DELAY);
}




void get_Imu(imu_datalar *data)
{
	float Roll = 0.0f;  // phi
	float Pitch = 0.0f; //theta
	float Yaw = 0.0f;

	float theta = 0.0f;
	float phi = 0.0f;




    MPU6050_read_data(&sensorData);


    //float Roll_Acc = atanf(sensorData.acc.Y / sensorData.acc.Z) *Rad_to_deg;
    //float Pitch_Acc = asinf(sensorData.acc.X / gravity_constant_value) *Rad_to_deg;

    /* 0-90 arası roll ve pitch verir.*/
    float Roll_Acc 	= 	atanf(sensorData.acc.Y / sqrt(pow(sensorData.acc.X,2)+pow(sensorData.acc.Z,2))) *Rad_to_deg;
    float Pitch_Acc = 	atanf(sensorData.acc.X /sqrt(pow(sensorData.acc.Y,2)+pow(sensorData.acc.Z,2))) *Rad_to_deg;
    float Yaw_Acc 	=  	atanf(sensorData.acc.Z/sqrt(pow(sensorData.acc.X,2) + pow(sensorData.acc.Z,2)))*Rad_to_deg;


   // sprintf(print_data,"Gyro Value Roll %.3f degree , Pitch %.3f degree, Yaw %.3f degree\r\n",Roll_Acc,Pitch_Acc,Yaw_Acc);
    //CDC_Transmit_FS(print_data, strlen(print_data));


	/* Transform body rates to Euler rate*/
    float Roll_Gyro 	= sensorData.gyro.X + tanf(theta) * (sinf(phi) + sensorData.gyro.Y + cosf(phi) * sensorData.gyro.Z);
    float Pitch_Gyro 	= 									cosf(phi) + sensorData.gyro.Y - sinf(phi) + sensorData.gyro.Z;
	double Yaw_Gyro 	= sensorData.gyro.Y * (sinf(phi) / cosf(theta)) + sensorData.gyro.Z * (cos(phi) / cos(theta));


	//printf(print_data,"Gyro Value Roll %.3f degree , Pitch %.3f degree, Yaw %.3f degree\r\n",Roll_Gyro,Pitch_Gyro,Yaw_Gyro);
	//CDC_Transmit_FS(print_data, strlen(print_data));
	/*
	 * Complementary filter
	 */
	 phi 	= (ALPHA * Roll_Acc + (1.0f - ALPHA) * (phi + SAMPLE_TIME_USB / 1000.0f) * Roll_Gyro) * Rad_to_deg;
	 theta 	= (ALPHA * Pitch_Acc + (1.0f - ALPHA) * (theta + SAMPLE_TIME_USB / 1000.0f) * Pitch_Gyro) * Rad_to_deg;
	 //sprintf(print_data,"Roll %.3f degree , Pitch %.3f degree, Yaw %.3f \r\n",phi,theta,Yaw_Acc);
	 //CDC_Transmit_FS(print_data, strlen(print_data));

	 data->pitch = Pitch_Acc;
	 data->roll = Roll_Acc;
	 //CDC_Transmit_FS(print_data, strlen(print_data));

	 data->ax = sensorData.acc.X;
	 data->ay = sensorData.acc.Y;
	 data->az = sensorData.acc.Z;
	 data->gx = sensorData.gyro.X;
	 data->gy = sensorData.gyro.Y;
	 data->gz = sensorData.gyro.Z;

	 /*
	  * Kalman filter:
	  * ->	"Optimal" method of choosing ALPHA (under certain assumptious..)
	  * ->	Can work with many different systems and system size (using matrices) !
	  * ->	Matrix form of "classical observer"
	  *
	  * x
	  *
	  *
	  *
	  */

}

