/*
 * attitude_control.cpp
 *
 *  Created on: 17 Oca 2023
 *      Author: FURKAN
 */



#include "attitude_controller.h"


float control_attitude(float dt, attitude_control_data *ctl_data);


/*
* --PITCH CONTROLLER---
*/
float pitch_controller(float dt, attitude_control_data *ctl_data){

	/* Calculate the error */
	float pitch_error = ctl_data.pitch_setpoint - ctl_data.pitch;

	/*  Apply P controller: rate setpoint from current error and time constant */
	_euler_rate_setpoint =  pitch_error / _tc;

	/* Transform setpoint to body angular rates (jacobian) */
	const float pitch_body_rate_setpoint_raw = cosf(ctl_data.roll) * _euler_rate_setpoint +
			cosf(ctl_data.pitch) * sinf(ctl_data.roll) * ctl_data.euler_yaw_rate_setpoint;
	_body_rate_setpoint = math::constrain(pitch_body_rate_setpoint_raw, -_max_rate_neg, _max_rate);

	return _body_rate_setpoint;


}

/*
 * --ROLL CONTROLLER---
 */
float roll_controller(float dt, attitude_control_data *ctl_data){

	/* Calculate the error */
	float roll_error = ctl_data.roll_setpoint - ctl_data.roll;

	/*  Apply P controller: rate setpoint from current error and time constant */
	_euler_rate_setpoint = roll_error / _tc;

	/* Transform setpoint to body angular rates (jacobian) */
	const float roll_body_rate_setpoint_raw = _euler_rate_setpoint - sinf(ctl_data.pitch) *
			ctl_data.euler_yaw_rate_setpoint;
	_body_rate_setpoint = math::constrain(roll_body_rate_setpoint_raw, -_max_rate, _max_rate);

	return _body_rate_setpoint;
}


/*
* --YAW CONTROLLER---
*/
float yaw_controller(float dt, attitude_control_data *ctl_data){

	/* Do not calculate control signal with bad inputs
		if (!(PX4_ISFINITE(ctl_data.roll) &&
		      PX4_ISFINITE(ctl_data.pitch) &&
		      PX4_ISFINITE(ctl_data.euler_pitch_rate_setpoint))) {

			return _body_rate_setpoint;
		}
*/
		float constrained_roll;
		bool inverted = false;

		/* roll is used as feedforward term and inverted flight needs to be considered */
		if (fabsf(ctl_data.roll) < math::radians(90.0f)) {
			/* not inverted, but numerically still potentially close to infinity */
			constrained_roll = math::constrain(ctl_data.roll, math::radians(-80.0f), math::radians(80.0f));

		} else {
			inverted = true;

			// inverted flight, constrain on the two extremes of -pi..+pi to avoid infinity
			//note: the ranges are extended by 10 deg here to avoid numeric resolution effects
			if (ctl_data.roll > 0.0f) {
				/* right hemisphere */
				constrained_roll = math::constrain(ctl_data.roll, math::radians(100.0f), math::radians(180.0f));

			} else {
				/* left hemisphere */
				constrained_roll = math::constrain(ctl_data.roll, math::radians(-180.0f), math::radians(-100.0f));
			}
		}

		constrained_roll = math::constrain(constrained_roll, -fabsf(ctl_data.roll_setpoint), fabsf(ctl_data.roll_setpoint));


		if (!inverted) {
			/* Calculate desired yaw rate from coordinated turn constraint / (no side forces) */
			_euler_rate_setpoint = tanf(constrained_roll) * cosf(ctl_data.pitch) * CONSTANTS_ONE_G / (ctl_data.airspeed <
					       ctl_data.airspeed_min ? ctl_data.airspeed_min : ctl_data.airspeed);

			/* Transform setpoint to body angular rates (jacobian) */
			const float yaw_body_rate_setpoint_raw = -sinf(ctl_data.roll) * ctl_data.euler_pitch_rate_setpoint +
					cosf(ctl_data.roll) * cosf(ctl_data.pitch) * _euler_rate_setpoint;
			_body_rate_setpoint = math::constrain(yaw_body_rate_setpoint_raw, -_max_rate, _max_rate);
		}

		if (!PX4_ISFINITE(_body_rate_setpoint)) {
			PX4_WARN("yaw rate sepoint not finite");
			_body_rate_setpoint = 0.0f;
		}

		return _body_rate_setpoint;
	}


}

}
