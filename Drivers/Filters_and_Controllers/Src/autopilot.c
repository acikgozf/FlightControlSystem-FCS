/*
 * Autopilot.c
 *
 *  Created on: 5 Oca 2023
 *      Author: FURKAN
 */


#include "Autopilot.h"

typedef struct{




}flight;


typedef enum
{
	attitude_hold_mode,
	pitch_hold__mode,
	climb_or_descent_dive_mode,
	landing_mode,
	take_off_mode,
	tfta,
	disccymmetrical_wings_mode

}flight_mode;


typedef enum{

	minimum_altitude_safety,
	automatic_stall_recovery


}flight_safety_mode;

typedef struct{
	float velocity_ms;
	float altitıde_m
	float pitch_angle_deg;




}attitude;


void check_ground_control_data();

void flight_task_otomatik(void);
bool update_tasks(void);

