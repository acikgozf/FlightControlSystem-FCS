/*
 * attitude_controller.h
 *
 *  Created on: 17 Oca 2023
 *      Author: FURKAN
 */

#ifndef FILTERS_AND_CONTROLLERS_ATTITUDE_CONTROLLER_H_
#define FILTERS_AND_CONTROLLERS_ATTITUDE_CONTROLLER_H_

#include <math.h>



typedef struct {
	float roll;
	float pitch;
	float yaw;
	float body_x_rate;
	float body_y_rate;
	float body_z_rate;
	float roll_setpoint;
	float pitch_setpoint;
	float yaw_setpoint;
	float euler_roll_rate_setpoint;
	float euler_pitch_rate_setpoint;
	float euler_yaw_rate_setpoint;
	float airspeed_min;
	float airspeed_max;
	float airspeed;
	float groundspeed;
	float groundspeed_scaler;
}attitude_control_data;

/**
	 * @brief Calculates both euler and body rate setpoints. Has different implementations for all body axes.
	 *
	 * @param dt Time step [s]
	 * @param ctrl_data Various control inputs (attitude, body rates, attitdue stepoints, euler rate setpoints, current speeed)
	 * @return Body rate setpoint [rad/s]
	 *
	 *
**/

float control_attitude(float dt, attitude_control_data *ctl_data);

/*
* --PITCH CONTROLLER---
*/
float pitch_controller(float dt, attitude_control_data *ctl_data);

/*
 * --ROLL CONTROLLER---
 */
float roll_controller(float dt, attitude_control_data *ctl_data);


/*
* --YAW CONTROLLER---
*/
float yaw_controller(float dt, attitude_control_data *ctl_data);

#endif /* FILTERS_AND_CONTROLLERS_ATTITUDE_CONTROLLER_H_ */
