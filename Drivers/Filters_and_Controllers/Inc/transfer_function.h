/*
 * Transfer_function.h
 *
 *  Created on: 12 May 2022
 *      Author: FURKAN
 */

#ifndef FILTERS_AND_CONTROLLERS_TRANSFER_FUNCTION_H_
#define FILTERS_AND_CONTROLLERS_TRANSFER_FUNCTION_H_

#define buf_size 255


#include "stm32f4xx_hal.h"


typedef struct Transfer_function{
	// initial conditions

	uint16_t y0 = 0;
	uint16_t yd0 = 0;
	uint16_t ud0 = 0;

	uint16_t ud[buf_size]  = {0};
	uint16_t y[buf_size]   = {0};
	uint16_t yd[buf_size]  = {0};
	uint16_t yyd[buf_size] = {0};

	HAL_RCC_GetHCLKFreq(void);


}TF;


#endif /* FILTERS_AND_CONTROLLERS_TRANSFER_FUNCTION_H_ */
