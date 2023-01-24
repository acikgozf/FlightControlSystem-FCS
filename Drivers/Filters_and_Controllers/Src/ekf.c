/*
 * EKF.c
 *
 *  Created on: Oct 13, 2022
 *      Author: FURKAN
 */
#include "EKF.h"

void KalmanRollPitch_init	(Kalman_filter *kal,float Pinit, float *Q, float *R){

	kal->phi_rad 	= 0.0f;	//Roll
	kal->theta_rad 	= 0.0f; //Pitch

	// P nin 4 eleman tutmasının sebebi acc_pitch, pred_pitch, acc_roll, pred_roll den dolayı
	kal->P[0] = Pinit;
	kal->P[1] = 0.0f;
	kal->P[2] = 0.0f;
	kal->P[3] = Pinit;


	/* model gurultusunun covariance matrisi (diagonal) pitch ve roll icin*/
	kal->Q[0] = Q[0];
	kal->Q[1] = Q[1];

	kal->R[0] = R[0];
	kal->R[1] = R[1];
	kal->R[2] = R[2];
}

void KalmanRollPitch_predict(Kalman_filter *kal,float *gyr_rps, float T){

	/* Extract measurement*/
	float pitch = gyr_rps[0];
	float roll 	= gyr_rps[1];
	float yaw 	= gyr_rps[2];

	/* Predict */

	/* Compute common trig term */

	float sin= sin(kal->phi_rad);
	float cos = cos(kal->phi_rad);
	float tan = tan(kal->theta_rad);

	/* x += x + T * f(x,u ) */

	//EULER RATES HESAPLAMA !!!
	kal->phi_rad 	= kal->phi_rad + T * (pitch + tan * ( roll *sin + yaw *cos));
	kal->theta_rad 	= kal->theta_rad + T *(roll * cos - yaw *sin);

	/*Recompute common trig term using new state estimates */
	sin = sin(kal->phi_rad);
	cos = cos(kal->phi_rad);
	float st = sin(kal->theta_rad);
	float ct = cos(kal->theta_rad);

	tan = st / ct;

	/*Jacobian of f(x,u)*/
	float A[4] = {tan * (q * cos - r * sin) , (r * cos + q * sin ) * (tan * tan + 1.0f) ,-(r * cos + q * so), 0.0f};

	/*Update covariance matrix P+ = P- + T*(AP- + P-*A + Q) */
	float Ptemp[4] = {	T*(kal->Q[0] + 2.0f*A[0]*kal->P[0] + A[1]*kal.P[1]+A[1]*kal->P[2]),
						T*(A[0]*kal->P[1] + A[2]*kal->P[0] + A[1]*kal->P[3] + A[3]*kal->P[1]),
						T*(A[0]*kal->P[2] + A[2]*kal->P[0] + A[1]*kal->P[3] + A[3]*kal->P[2]),
						T*(kal->Q[1] + A[2]*kal->P[1] + A[2]*kal->P[2] + 2.0f*A[3]*kal->P[3])
					};

	kal->P[0] = kal->P[0] + Ptemp;
	kal->P[1] = kal->P[1] + Ptemp;
	kal->P[2] = kal->P[2] + Ptemp;
	kal->P[3] = kal->P[3] + Ptemp;

	}

}

void KalmanRollPitch_update	(Kalman_filter *kal,float *acc_msp2){


	/*Extract measurements*/
	float ax = acc_msp2[0];
	float ay = acc_msp2[1];
	float az = acc_msp2[2];

	/*Conpute common trig terms*/
	float sp= sin(kal->phi_rad);
	float cp = cos(kal->phi_rad);
	float st= sin(kal->theta_rad);
	float ct = cos(kal->theta_rad);

	/*Output function h(x,u) */

	float h[3] = {g* st, -g*ct*sp, -g*ct*cp);

	/*Jacobian of h(x,u) */
	float C[6] = {0.0f, g*ct,
				  -g*cp*ct, g*sp*st,
				   g*sp*st, g*cp*st};


	/*Kalman Gain K = P * C' / (C * P * C' * R) */

	float G[9] = {kal->P[3]*C[1]*C[1] + kal->R[0],
					C[1]*C[2]*kal->P[2] + C[1]*C[3]*kal->P[3],
					C[1]*(C[2]*kal->P[1] + C[3]*kal->P[3]),
					kal->R[1]  + C[2]*(C[2]*kal->P[0] + C[3]*kal->P[2]) + C[3]*(C[2]*kal->P[1] + C[3]*kal->P[3]),
					C




	};




	};









}

