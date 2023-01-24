/*
 * PID.c
 *
 *  Created on: 18 Nis 2022
 *      Author: FURKAN
 */
#include <pid.h>

/*
 *
 * The requirements is rotation angle and angular rate.
 *
 * According to target
 *
 * */

void PIDController_Init(PIDController *pid ){

//Clear controller variables
	pid->integrator = 0.0f;
	pid->prevError = 0.0f;
	pid->differentiator = 0.0f;
	pid->prevMeasuremet = 0.0f;
	pid->out = 0.0f;

}




float PIDContoller_Update(PIDController *pid, float setpoint ,float measurement){

	//Error signal
	float error = setpoint - measurement;

	//Proportional
	float proportional = pid->Kp *error;
	//Integral
	pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error * pid->prevError);

	//Anti-wind-up via dynamic integrator clamping
	float limMinInt,limMaxInt;

	//Compute integration limits
	if (pid->limMax > proportional){
		limMaxInt = pid->limMax - proportional;
		}
	else
		limMaxInt = 0.0f;

	if (pid->limMin > proportional){
		limMaxInt = pid->limMin - proportional;
		}
	else
		limMinInt = 0.0f;

	//Clamp integrator
	if (pid->integrator >limMaxInt)
	{
		pid->integrator = limMaxInt;
	}
	else if (pid->integrator < limMinInt)
	{
		pid->integrator = limMinInt;

	}
	//Derivative (band-limited-differantior)
	pid->differentiator = (2.0f + pid->Kd *(measurement - pid->prevMeasuremet) // Note:derivate on measurement
							+ (2.0f * pid->tau - pid->T) * pid->differentiator)
							/ (2.0f * pid->tau + pid->T);

	//Compute output and apply limits
	pid->out = proportional  + pid->integrator + pid->differentiator;

	if(pid->out > pid->limMax)
	{
		pid->out = pid->limMax;
	}
	else if (pid->out < pid->limMin)
	{
		pid->out = pid->limMin;

	}

	//Store error and measurement for later use
	pid->prevError = error;
	pid->prevMeasuremet = measurement;

	//return contoller output
	return pid->out;



}
