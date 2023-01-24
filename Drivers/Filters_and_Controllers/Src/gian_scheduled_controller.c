/*
 * Gian_scheduled_controller.c
 *
 *  Created on: 21 Ara 2022
 *      Author: FURKAN
 */
#include "Gian_scheduled_controller.h"

//Calculate the control signal
void cal_GainScheduled(GS_obj *obj,float Va){

	//Calculate the weigthing parameters lamda_H, lamda_M, lamda_L based on the airspeed Va

	if(Va > Va_H){
		obj->lambda_H = 1.0;
		obj->lambda_M = 0.0;
		obj->lambda_L = 0.0;


	}
	else if (Va < Va_H && Va > (Va_M + delta)) {

		obj->lambda_H = (Va - Va_M - delta) /(Va_H - Va_M - 2 * delta);
		obj->lambda_M = 1.0 - obj->lambda_H;
		obj->lambda_L = 0.0;

	}
	else if (Va > (Va_M - delta) && Va < (Va_M + delta)) {

		obj->lambda_H = 0.0;
		obj->lambda_M = 1.0;
		obj->lambda_L = 0.0;

	}
	else if (Va < (Va_M - delta) && Va > Va_L) {

		obj->lambda_H = 0.0;
		obj->lambda_M = (Va - Va_L - delta) / (Va_M - Va_L -2 * delta);
		obj->lambda_L = 1.0 - obj->lambda_M;
	}
	else if (Va < Va_L){
		obj->lambda_H = 0.0;
		obj->lambda_M = 0.0;
		obj->lambda_L = 1.0;
	}

	//Calculate the associated incremental control signal based on each airspeed condition

	float u_H = obj->Kp_H * (-obj->y + obj->yPast) + obj->Ki_H * delta_t * (obj->r - obj->y);
	float u_M = obj->Kp_M * (-obj->y + obj->yPast) + obj->Ki_M * delta_t * (obj->r - obj->y);
	float u_L = obj->Kp_L * (-obj->y + obj->yPast) + obj->Ki_L * delta_t * (obj->r - obj->y);

	//Calculate the control signal to be imlemented
	obj->u = obj->uPast + lambda_H *u_H + lambda_M*u_M + lambda_L*u_L;

	//Anti-windup mechanism
	if(obj->u > obj->u_max)
	{
		obj->u = obj->u_max;
	}
	else(obj->u < obj->u_min){

		obj->u = obj->u_min;
	}
	//Updatte past variables
	obj->rPast = obj->r;
	obj->yPast = obj->r;
	obj-urPast = obj->r;


}



