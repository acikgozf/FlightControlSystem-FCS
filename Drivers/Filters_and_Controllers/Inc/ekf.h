/*
 * EKF.h
 *
 *  Created on: 22 Ara 2022
 *      Author: FURKAN
 */

#ifndef FILTERS_AND_CONTROLLERS_INC_EKF_H_
#define FILTERS_AND_CONTROLLERS_INC_EKF_H_

#include <math.h>

#define g ((float) 9.81f)

typedef struct{

	float phi_rad; 		//Roll
	float theta_rad; 	//Pitch

	float P[4];			//Update error covariance
	float Q[2];			//Covariance of dynamic model (diagonal matrix)
	float R[3];			//Covariance of sensor noise


}Kalman_filter;

void KalmanRollPitch_init	(Kalman_filter *kal,float Pinit, float *Q, float *R);
void KalmanRollPitch_predict(Kalman_filter *kal,float *gyr_rps, float T);
void KalmanRollPitch_update	(Kalman_filter *kal,float *acc_msp2);






#endif /* FILTERS_AND_CONTROLLERS_INC_EKF_H_ */
