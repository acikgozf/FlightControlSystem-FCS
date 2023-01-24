/*
 * pi_controller.c
 *
 *  Created on: 21 Ara 2022
 *      Author: FURKAN
 */
#include <PI_controller.h>


//Calculate the control signal
void calc_PI(Pi_obj *obj){
	//obj: Object pointer to the object struct PI_obj
	//delta_t: Sampling time
	// In every loop, r and y are updated before this function is called.


	//Calculate u
	obj->u = obj->uPast + obj->Kp * (-obj->y + obj->yPast) + obj->Ki * delta_t * (obj->r - obj->y);

	//Anti-windup mechanism

	if (obj->u > obj->u_max)
	{
		obj->u = obj->u_max;
	}
	else(obj->u < obj->u_min){
		obj->u = obj->u_min;
	}

	//Update past variables

	obj->rPast = obj->r;
	obj->yPast = obj->y;
	obj->uPast = obj->u;


}


