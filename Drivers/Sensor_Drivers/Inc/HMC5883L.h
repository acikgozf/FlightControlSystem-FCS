/*
 * HMC5883L.h
 *
 *  Created on: 4 Eyl 2022
 *      Author: FURKAN
 */

#ifndef SENSOR_DRIVERS_INC_HMC5883L_H_
#define SENSOR_DRIVERS_INC_HMC5883L_H_

#include "stdint.h"

#define HMC5883L_I2C_ADRESS_ 	( 0x1E <<1 )

#define CONF_REG_A				( 0 )
#define CONF_REG_B				( 1 )
#define MODE_REG				( 2 )
#define DATA_OUTPUT_X_MSB_REG	( 3 )
#define DATA_OUTPUT_X_LSB_REG	( 4 )
#define DATA_OUTPUT_Z_MSB_REG	( 5 )
#define DATA_OUTPUT_Z_LSB_REG	( 6 )
#define DATA_OUTPUT_Y_MSB_REG	( 7 )
#define DATA_OUTPUT_Y_LSB_REG	( 8 )
#define STATUS_REG				( 9 )
#define IDEN_REG_A				( 10 )
#define IDEN_REG_B				( 11 )
#define IDEN_REG_C				( 12 )

typedef enum{
		OK,
		ERROR
}sensor_status;


typedef enum{
	CONT_MODE,
	SING_MODE,
	IDLE_MODE
}HMC5883L_Mode;


typedef struct{
	uint16_t dev;
	uint8_t data[2];

}HMC5883L;




sensor_status HMC5883L_Self_test(HMC5883L *dev, );
sensor_status HMC5883L_Initialise(HMC5883L *dev,);
sensor_status HMC5883L_Read_data(HMC5883L *dev,);




#endif /* SENSOR_DRIVERS_INC_HMC5883L_H_ */
