/*
 * neo_m8n_gps.h
 *
 *  Created on: 19 Oca 2023
 *      Author: FURKAN
 */

#ifndef SENSOR_DRIVERS_INC_NEO_M8N_GPS_H_
#define SENSOR_DRIVERS_INC_NEO_M8N_GPS_H_


/*
 * ---NMEA Protocol---
 *
 * All explainiton about NEO M8N
 *
 * GPGGA (Global Positioning System Fix Data) and contains time, latitude, longitude, altitude and number of satellites used in calculation
 * GPGSV (GPS Satellites in View)  			  and contains satellite info
 * GPRMC (Recommended Minimum Data)		    MİNİMUM RECONMENDTED !!! :data,time reliability , latitude ,longitude ,speed,direction, date,
 * GNVTG ?
 * GNGLL ?
 *
 * For example data explanition:
 *
 *
 * $GNRMC,132648.00,A,3733.34122,N,12655.65876,E,1.116,,290719,,,A*6A
 * current time:132648.00
 * A(Active) means reliable data. if it is to be V(void) it is invalid data.
 * Latitude : 3733.34122 --- 37 degree and 33.34122 minutes dereceye cevirmek icin (33.34122 / 60) = 0.555687
 * N(North)longitude
 * Longitude : 12655.65876 --- 126 degree and 55.65876 minutes
 * E(East) longitude
 * Speed :1.116 knots. to convert to KM, multiply by 1.8.
 * Current date: 290719(DDMMYY)
 * A is not analyzed by myself.
 * Checksum : 6A
 *
 * $GNGGA,132648.00,A,3733.34122,N,12655.65876,E,1,04,10.64,34.2M,18.3M,,4D
 *
 * current time:132648.00
 * A(Active) means reliable data. if it is to be V(void) it is invalid data.
 * Latitude : 3733.34122 --- 37 degree and 33.34122 minutes dereceye cevirmek icin (33.34122 / 60) = 0.555687
 * N(North)longitude
 * Longitude : 12655.65876 --- 126 degree and 55.65876 minutes
 * E(East) longitude
 * 1 is basic satellite is avaible ,0 is not avaible, 2 menas using DGPS
 * 04 means number of satellites used to calculate the coordinates
 * 10.64 is horizontal dilution of precision information. ??
 * 34.2,M orthomettric heigh which mean sea level altitude in M units. And can be compared with the barometric altitude using barometer.
 * 18.2,M geoidal separation
 * *4D checksum
 *
 *
 */
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"




typedef struct{

	unsigned char class;
	unsigned char id;
	unsigned short length;

	unsigned int itow;
	signed int longitude;
	signed int latitude;
	signed int heigh;
	signed int hmsl;
	unsigned int hacc;
	unsigned int vacc;

	double longitude_f_64;
	double latitude_f_64;


}m8n_ubx_nav_posllh;




typedef enum{
	config_ok,
	config_error

}config_status;

typedef enum{
	crc_ok,
	crc_error

}checkum_status;

typedef enum{

	parsing_ok,
	parsing_error

}parsing_status;


void m8n_ubx_init(void);
void m8n_ubx_take_data(void);
config_status m8n_ubx_default_config(unsigned char *data, unsigned char len);
checkum_status m8n_ubx_checksum_check(unsigned char *data, unsigned char len);
parsing_status m8n_ubx_parsing(unsigned char *data, m8n_ubx_nav_posllh *posllh);


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);



#endif /* SENSOR_DRIVERS_INC_NEO_M8N_GPS_H_ */
