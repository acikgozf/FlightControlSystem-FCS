/*
 * bldc_motor_driving.c
 *
 *  Created on: Jan 21, 2023
 *      Author: FURKAN
 */

#include "bldc_motor_driving.h"


extern TIM_HandleTypeDef htim2;



void esc_initialize(void){
	htim2.Instance->CCR1 = 21000;
	htim2.Instance->CCR2 = 21000;
	HAL_Delay(7000);

	htim2.Instance->CCR1 = 10500;
	htim2.Instance->CCR2 = 10500;
	HAL_Delay(8000);

}
void esc_control(uint16_t rc_level){



	htim2.Instance->CCR1 = 10500 +(rc_level - 1000) * 10.5; //rc_level 1000-2000 arasında olmalı
	htim2.Instance->CCR2 = 10500 +(rc_level - 1000) * 10.5;


}
