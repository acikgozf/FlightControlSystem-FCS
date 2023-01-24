/*
 * PID.h
 *
 *  Created on: 18 Nis 2022
 *      Author: FURKAN
 */

#ifndef SENSOR_DRIVERS_INC_PID_H_
#define SENSOR_DRIVERS_INC_PID_H_


typedef struct {
	//Controller gains
	float Kp;
	float Ki;
	float Kd;

	//Derivative low-pass filter time constant
	float tau;
	//Output limits
	float limMin;
	float limMax;
	//Sample time (in seconds)

	float T;


	//Controller "memory"
	float integrator;
	float prevError;		//required for integrator
	float differentiator;
	float prevMeasuremet;	//requıired for differentiator

	//controller output
	float out;


}PIDController;

void PIDController_Init(PIDController *pid );
float PIDContoller_Update(PIDController *pid, float setpoint ,float measurement);

#endif /* SENSOR_DRIVERS_INC_PID_H_ */
