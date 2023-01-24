/*
 * Servo_control.h
 *
 *  Created on: Dec 8, 2022
 *      Author: FURKAN
 */

#ifndef INC_SERVO_CONTROL_H_
#define INC_SERVO_CONTROL_H_

#include <pid.h>
#include "mpu6050.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"


typedef struct{
	TIM_HandleTypeDef htim1;
	PIDController *pid;
	float pwm_degeri;


}servo_control;

void control_init (void);
void control_start (void);



#endif /* INC_SERVO_CONTROL_H_ */
