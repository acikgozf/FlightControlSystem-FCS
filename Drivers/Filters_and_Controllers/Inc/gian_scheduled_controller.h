/*
 * Gian_scheduled_controller.h
 *
 *  Created on: 21 Ara 2022
 *      Author: FURKAN
 */

#ifndef GIAN_SCHEDULED_CONTROLLER_H_
#define GIAN_SCHEDULED_CONTROLLER_H_

#define Va_H 15 	//define high airspeed limit
#define Va_M 10 	//define medium airspeed limit
#define Va_L 7 		//define low airspeed limit
#define delta 0.5	//define boundary

//Define object for the Gain scheduled controller struct
typedef struct{
	float Kp_H;		//Proportional gain for high airpeed
	float Ki_H;		//Proportional gain for high airpeed
	float Kp_M;		//Proportional gain for medium airpeed
	float Ki_M;		//Proportional gain for medium airpeed
	float Kp_L;		//Proportional gain for low airpeed
	float Ki_L;		//Proportional gain for low airpeed

	float u;		//Control signal
	float r;		//Reference signal
	float y;		//Output signal

	float uPast;	//Past value of the control signal
	float rPast;	//Past value of the reference signal
	float yPast;	//Past value of the output signal

	float u_max;	//The maximum limit of the control signal
	float u_min;	//The minimum limit of the control signal

	float lambda_H 	//Weighting parameter (High airspeed)
	float lambda_M 	//Weighting parameter (Medium airspeed)
	float lambda_L 	//Weighting parameter (Low airspeed)

}GS_obj;


//Calculate the control signal
void cal_GainScheduled(GS_obj *obj,float Va);



#endif /* GIAN_SCHEDULED_CONTROLLER_H_ */
