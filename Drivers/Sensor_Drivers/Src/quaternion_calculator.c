/*
 * quaternion_calculator.c
 *
 *  Created on: 23 Oca 2023
 *      Author: FURKAN
 */

#include "quaternion_calculator.h"



// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration and rotation rate to produce a quaternion-based estimate of relative
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!




void MadgwickQuaternionUpdate(float ax, float ay, float az, float gyrox, float gyroy, float gyroz,float deltat,float zeta,float beta )
{


	float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
	float norm;                                               // vector norm
	float f1, f2, f3;                                         // objetive funcyion elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
	float qDot1, qDot2, qDot3, qDot4;
	float hatDot1, hatDot2, hatDot3, hatDot4;
	float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

	// Auxiliary variables to avoid repeated arithmetic
	float _halfq1 = 0.5f * q1;
	float _halfq2 = 0.5f * q2;
	float _halfq3 = 0.5f * q3;
	float _halfq4 = 0.5f * q4;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
//	float _2q1q3 = 2.0f * q1 * q3;
//	float _2q3q4 = 2.0f * q3 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Compute the objective function and Jacobian
	f1 = _2q2 * q4 - _2q1 * q3 - ax;
	f2 = _2q1 * q2 + _2q3 * q4 - ay;
	f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
	J_11or24 = _2q3;
	J_12or23 = _2q4;
	J_13or22 = _2q1;
	J_14or21 = _2q2;
	J_32 = 2.0f * J_14or21;
	J_33 = 2.0f * J_11or24;

	// Compute the gradient (matrix multiplication)
	hatDot1 = J_14or21 * f2 - J_11or24 * f1;
	hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
	hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
	hatDot4 = J_14or21 * f1 + J_11or24 * f2;

	// Normalize the gradient
	norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
	hatDot1 /= norm;
	hatDot2 /= norm;
	hatDot3 /= norm;
	hatDot4 /= norm;

	// Compute estimated gyroscope biases
	gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
	gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
	gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

	// Compute and remove gyroscope biases
	gbiasx += gerrx * deltat * zeta;
	gbiasy += gerry * deltat * zeta;
	gbiasz += gerrz * deltat * zeta;
//	gyrox -= gbiasx;
//	gyroy -= gbiasy;
//	gyroz -= gbiasz;

	// Compute the quaternion derivative
	qDot1 = -_halfq2 * gyrox - _halfq3 * gyroy - _halfq4 * gyroz;
	qDot2 =  _halfq1 * gyrox + _halfq3 * gyroz - _halfq4 * gyroy;
	qDot3 =  _halfq1 * gyroy - _halfq2 * gyroz + _halfq4 * gyrox;
	qDot4 =  _halfq1 * gyroz + _halfq2 * gyroy - _halfq3 * gyrox;

	// Compute then integrate estimated quaternion derivative
	q1 += (qDot1 -(beta * hatDot1)) * deltat;
	q2 += (qDot2 -(beta * hatDot2)) * deltat;
	q3 += (qDot3 -(beta * hatDot3)) * deltat;
	q4 += (qDot4 -(beta * hatDot4)) * deltat;

	// Normalize the quaternion
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;

/*
	Pitch = atan2f(2.0f * (q2*q3 + q1*q4), q1*q1 + q2*q2 - q3*q3 - q4*q4);
	Roll  = -asinf(2.0f * (q2*q4 - q1*q3));
	Yaw   = atan2f(2.0f * (q1*q2 + q3*q4), q1*q1 - q2*q2 - q3*q3 + q4*q4);

	Pitch *= PI;
	Roll  *= PI;
	Yaw   *= PI;


	if(Yaw>=0)
		Yaw = 360.f - Yaw;
	else
		Yaw = -Yaw;


	if(Pitch>=0)
		Pitch = 180.f - Pitch;
	else
		Pitch = -(Pitch + 180.f);

*/
	Yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
	Pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
	Roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);


	Pitch *= 180.0f / PI;
	Yaw   *= 180.0f / PI;
    Roll  *= 180.0f / PI;
    uint8_t quaternion_buf[36];

    sprintf(quaternion_buf,"Roll %.3f degree , Pitch %.3f degree, Yaw %.3f degree\r\n",Roll,Pitch,Yaw);
    CDC_Transmit_FS(quaternion_buf, strlen(quaternion_buf));



}



