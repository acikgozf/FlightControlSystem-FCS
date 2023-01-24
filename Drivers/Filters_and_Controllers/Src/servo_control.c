/*
 * Servo_control.c
 *
 *  Created on: Dec 8, 2022
 *      Author: FURKAN
 */

#include <servo_control.h>
#include "GPS.h"


extern TIM_HandleTypeDef htim1;

servo_control *elevator;
servo_control *rudder;
servo_control *flap_sag;
servo_control *flap_sol;

static imu_datalar data;

/*
 * GPS Variables
 */

extern UART_HandleTypeDef huart3;


/*
	  position 0 1.5 pulse
	  position 90 2 pulse
	  position -90 1 pulse

 * duty cycle CCRx'e yazıcaz. Frekans 20ms(50Hz). 0.5ms 20ms'nin  2.5% yapar
 * Bizim ARR degerimiz 1000 yani 2.5% 25 yapar.
 *
 * 0.5ms 0 derece yapar.ARR 25
 * 1.5ms 90 derece yapar.ARR 75
 */

/* ARR 25 0 derece
 * 	   50 45
 *	   75 90
	   100 135
 */


/* Servolar : Aerlion (&a) ,Elevator(&e) , Rudder (&r)
 *
 * Aerlion is used to deflect ROLL
 * Elevator is used to deflect PİTCH
 * Rudder is used to deflect YAW
 *
 *
 * lateral axis (roll and yaw)
 * longitudinal axis (pitch)
 *
 *
 */

void control_init (void){

	flap_sol->pid->Kd = 1.8;
	flap_sol->pid->Ki = 10;
	flap_sol->pid->Kp = 8;
	PIDController_Init(flap_sol->pid);

	flap_sag->pid->Kd = 1.8;
	flap_sag->pid->Ki = 10;
	flap_sag->pid->Kp = 8;
	PIDController_Init(flap_sag->pid);

	rudder->pid->Kd = 1.8;
	rudder->pid->Ki = 10;
	rudder->pid->Kp = 8;
	PIDController_Init(rudder->pid);

	elevator->pid->Kd = 1.8;
	elevator->pid->Ki = 10;
	elevator->pid->Kp = 8;
	PIDController_Init(elevator->pid);


}
void control_start (void){


	uint8_t print_data[100];
	char get_gps_data[30];
	control_init();

	get_Imu(&data);


	sprintf(print_data,"%.3f,%.3f",data.roll,data.pitch);
	//HAL_UART_Receive(&huart3,&get_gps_data, 1, HAL_MAX_DELAY);
	//sprintf(print_data,"%C \r\n",data.roll,data.pitch,altitude,location,zaman);
	CDC_Transmit_FS(print_data, strlen(print_data));


	//PIDContoller_Update(pid, setpoint, measurement) //PID for sol aerlion
	//PIDContoller_Update(pid, setpoint, measurement)	//PID for sol aerlion
	//PIDContoller_Update(pid, setpoint, measurement)	//PID for rudder
	//float elevator_out = PIDContoller_Update(elevator->pid, 60, data.pitch);	//PID for elevator
;
	htim1.Instance->CCR1 =data.pitch;
//	htim1.Instance->CCR1 =50;
//	htim1.Instance->CCR1 =75;


	/*
	if(flap_sag->ImuData.roll != 0 && flap_sag->ImuData.roll > 0)
	{
		float roll = flap_sag->ImuData.roll;


				flap_sag->htim1.Instance->CCR1 = roll + 25;

	}


*/
//	htim1.Instance->CCR1 = PIDContoller_Update(&pid,40,flap_sag->ImuData.pitch);


	//flap_sag->
	//htim1.Instance->CCR1 =
}
