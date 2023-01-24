/*
 * LPF.h
 *
 *  Created on: 28 Nis 2022
 *      Author: FURKAN
 */

#ifndef FILTERS_AND_CONTROLLERS_INC_LPF_H_
#define FILTERS_AND_CONTROLLERS_INC_LPF_H_

#include "stdint.h"



typedef struct
{
	uint16_t ax;
	uint16_t ay;
	uint16_t az;
	uint16_t gx;
	uint16_t gy;
	uint16_t gz;

}Imu_Data;

uint16_t * Impulse_response [3] = {LPF,BPF,HPF}


void low_pass_filter(Imu_Data * data, Impulse_response);






#endif /* FILTERS_AND_CONTROLLERS_INC_LPF_H_ */
