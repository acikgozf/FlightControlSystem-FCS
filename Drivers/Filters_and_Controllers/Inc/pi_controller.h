/*
 * pi_controller.h
 *
 *  Created on: 21 Ara 2022
 *      Author: FURKAN
 */

#ifndef PI_CONTROLLER_H_
#define PI_CONTROLLER_H_

//Define objcet for the PI contoller struct

typedef struct {

	float Kp;
	float Ki;
	float u;		//control signal
	float r;		//reference signal
	float y;		//measured output
	float uPast;
	float rPast;
	float yPast;
	float u_max;
	float u_min;

}Pi_obj;


void calc_PI(Pi_obj *obj);








#endif /* PI_CONTROLLER_H_ */
