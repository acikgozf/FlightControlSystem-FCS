/*
 * quaternion_calculator.h
 *
 *  Created on: 23 Oca 2023
 *      Author: FURKAN
 */

#ifndef SENSOR_DRIVERS_INC_QUATERNION_CALCULATOR_H_
#define SENSOR_DRIVERS_INC_QUATERNION_CALCULATOR_H_

#include "usbd_cdc_if.h"
#include "math.h"

 float Roll;
 float Pitch;
 float Yaw;

typedef struct{
	float q[4];

}get_quaternion;

#define PI 3.14159265358979323846f



void MadgwickQuaternionUpdate(float ax, float ay, float az, float gyrox, float gyroy, float gyroz,float deltat,float zeta,float beta);

#endif /* SENSOR_DRIVERS_INC_QUATERNION_CALCULATOR_H_ */
