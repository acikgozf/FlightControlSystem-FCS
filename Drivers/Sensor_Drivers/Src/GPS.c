/*
 * GPS.c
 *
 *  Created on: 6 Eyl 2022
 *      Author: FURKAN
 */


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "main.h"

#include "GPS.h"
char UART_ReceivedChar;
extern UART_HandleTypeDef huart5;

/**  --------------------------- INTERNAL FUNCTIONS ---------------------------  **/

/**
 * INTERNAL FUNCTION
 *
 * @brief Tokenize given "str" string based on characters in "delim" string
 * Difference between strtok(...) function and this one is that,
 *  strtoke(...) takes empty strings between consecutive "delim" characters into account, too.
 *
 * Found on: https://github.com/lamik/GPS_NEO6_STM32_HAL/blob/master/Src/gps_neo6.c
 *
 * @param str: pointer to cstring to tokenize
 * @param delim: pointer to cstring for specifying delimiter characters
 *
 * @retval (char *) Pointer to the beginning of tokenized string
*/
char* strtoke(char *str, const char *delim)
{
        static char *start = NULL; /* stores string str for consecutive calls */
        char *token = NULL; /* found token */

        /* assign new start in case */
        if (str) start = str;

        /* check whether text to parse left */
        if (!start) return NULL;

        /* remember current start as found token */
        token = start;

        /* find next occurrence of delim */
        start = strpbrk(start, delim);

        /* replace delim with terminator and move start to follower */
        if (start) *start++ = '\0';

        /* done */
        return token;
}


/**
 * INTERNAL FUNCTION
 *
 * @brief Calculate or extract useful information from received NMEA messages
 *
 * @param gps: Pointer to GPS configuration and received-information struct
 * 		(used to store info and handle Neo6 GPS device)
 *
 * @retval Status Code
 *
*/
uint8_t calc_info(NEO6 *gps)
{
	// Parse every received message
        for (uint8_t m = 0; m < GPS_BUFFER_SIZE; m++){

		struct NEO6_ParsedInfo info = {
			{
				0,     // latitude value
				'0',   // latitude direction
				0,     // longtitude value
				'0',   // longtitude direction
				0      // altitude
			},
			"",            // UTC time
			"",            // UTC date
			0              // Quality of info
		};


		char *message = gps->com.messages_buffer[m];
		// printf("%s\n\r",  message);
		NMEA_MessageParse(message, &info);

		gps->info.quality = info.quality;
		if (info.quality){
			gps->info.pos.lat = info.pos.lat;
			gps->info.pos.lat_dir = info.pos.lat_dir;

			gps->info.pos.lon = info.pos.lon;
			gps->info.pos.lon_dir = info.pos.lon_dir;

			sprintf(gps->info.utc_time, "%s", info.utc_time);

			if (strlen(info.date))
				sprintf(gps->info.date, "%s", info.date);

			if (info.pos.alt)
				gps->info.pos.alt = info.pos.alt;
			break;
		}
	}

	gps->com.buffer_tail = 0;
        gps->com.message_tail = 0;
        return 0;
}

/**
 * INTERNAL FUNCTION
 *
 * Parse GPGGA function
 *
*/
void NMEA_GPGGAParse(struct NEO6_ParsedInfo *info)
{
	char *parsed_token;

	// Time of fix
	parsed_token = strtoke(NULL, ",*");
	if (strlen(parsed_token) > 0){
		uint32_t time = atoi(parsed_token);
		sprintf(info->utc_time, "%02lu:%02lu:%02lu", (time/10000) % 100, (time / 100) % 100, time % 100);
	}

	// Latitude (value and direction)
	parsed_token = strtoke(NULL, ",*");
	if (strlen(parsed_token) > 0){
		// latitude value
		double lat_deg = (atoi(parsed_token) / 100) % 100;
		double lat_min = atof(parsed_token) - lat_deg*100;
		info->pos.lat = lat_deg + lat_min / 60;

		// latitude direction
		parsed_token = strtoke(NULL, ",*");
		info->pos.lat_dir = *parsed_token;

	} else strtoke(NULL, ",*"); // jump over empty latitude direction

	// Longtitude (value and direction)
	parsed_token = strtoke(NULL, ",*");
	if (strlen(parsed_token) > 0) {
		// longtitude value
		double lon_deg = (atoi(parsed_token) / 100) % 100;
		double lon_min = atof(parsed_token) - lon_deg*100;
		info->pos.lon = lon_deg + lon_min / 60;

		// longtitude direction
		parsed_token = strtoke(NULL, ",*");
		info->pos.lon_dir = *parsed_token;

	} else strtoke(NULL, ",*"); // jump over empty longtitude direction

	// Quality of fix
	parsed_token = strtoke(NULL, ",*");
	if (strlen(parsed_token) > 0) {
		info->quality = (uint8_t)atoi(parsed_token);
	}

	// Number of sattellites
	strtoke(NULL, ",*");

	// HDOP
	strtoke(NULL, ",*");

	// Altitutde
	parsed_token = strtoke(NULL, ",*");
	if (strlen(parsed_token) > 0) {
		info->pos.alt = atof(parsed_token);
	}
	return ;
}

/**
 * INTERNAL FUNCTION
 *
 * Parse GPRMC message
*/
void NMEA_GPRMCParse(struct NEO6_ParsedInfo *info)
{
	char *parsed_token;

	// Time of FIX
	parsed_token = strtoke(NULL, ",*");
	if (strlen(parsed_token) > 0) {
		uint32_t time = atoi(parsed_token);
		sprintf(info->utc_time, "%02lu:%02lu:%02lu", (time/10000) % 100, (time / 100) % 100, time % 100);
	}

	// Quality of Data
	parsed_token = strtoke(NULL, ",*");
	if ((strlen(parsed_token) > 0)){
		if (!strcmp(parsed_token, "V"))
			info->quality = 0;
		else
			info->quality = 1;
	}

	// Latitude (value and direction)
	parsed_token = strtoke(NULL, ",*");
	if (strlen(parsed_token) > 0){
		// latitude value
		double lat_deg = (atoi(parsed_token) / 100) % 100;
		double lat_min = atof(parsed_token) - lat_deg*100;
		info->pos.lat = lat_deg + lat_min / 60;

		// latitude direction
		parsed_token = strtoke(NULL, ",*");
		info->pos.lat_dir = *parsed_token;

	} else strtoke(NULL, ",*"); // jump over empty latitude direction

	// Longtitude (value and direction)
	parsed_token = strtoke(NULL, ",*");
	if (strlen(parsed_token) > 0) {
		// longtitude value
		double lon_deg = (atoi(parsed_token) / 100) % 100;
		double lon_min = atof(parsed_token) - lon_deg*100;
		info->pos.lon = lon_deg + lon_min / 60;

		// longtitude direction
		parsed_token = strtoke(NULL, ",*");
		info->pos.lon_dir = *parsed_token;

	} else strtoke(NULL, ",*"); // jump over empty longtitude direction

	// Speed over ground
	strtoke(NULL, ",*");

	// Course over ground
	strtoke(NULL, ",*");

	// UTC date of FIX
	parsed_token = strtoke(NULL, ",*");
	if (strlen(parsed_token) > 0) {
		uint32_t date = atoi(parsed_token);
		sprintf(info->date, "%02lu.%02lu.20%lu", (date / 10000) % 100, (date / 100) % 100, date % 100);
	}

	return ;

}


/** --------------------------- LIBRARY FUNCTIONS --------------------------- **/

/**
 * @brief Receive NEO6 GPS information only 1 char over UART; internal function
 *
 * @param gps: Pointer to GPS configuration and received-information struct
 * 		(used to store info and handle Neo6 GPS device)
 *
 * @retval Status Code(From HAL or GPS)
*/
uint8_t NEO6_UART_ReceiveChar(NEO6 *gps)
{
        if (gps == NULL)
                return GPS_ERR_NULL_PTR;

        uint8_t response;

        if (gps->com.buffer_tail < 0){
                // buffer is full
                // calculate info and clear the buffer
                calc_info(gps);
                response = (uint8_t)GPS_BUF_FULL;
        }
        else {
                if (UART_ReceivedChar == 13){
                        // if carriage return (\r) is received then it means the end of the message
                        gps->com.messages_buffer[gps->com.buffer_tail][gps->com.message_tail] = '\0';

                        if (gps->com.buffer_tail < GPS_BUFFER_SIZE - 1)
                                gps->com.buffer_tail++;
                        else
                                gps->com.buffer_tail = -1;

                        gps->com.message_tail = 0;

                        response = GPS_MSG_CPLT;

                }
                else if (UART_ReceivedChar == 10 || UART_ReceivedChar == 0){
                        // ignore the line feed (\n) character
                        response = (uint8_t)GPS_CHR_RECEIVED;

                }
                else {
                        gps->com.messages_buffer[gps->com.buffer_tail][gps->com.message_tail] = UART_ReceivedChar;
                        gps->com.message_tail++;

                        response = (uint8_t)GPS_CHR_RECEIVED;

                }
        }

        HAL_UART_Receive_IT(gps->com.uart, (uint8_t*)&UART_ReceivedChar, 1);


        return response;

}


/**
 * @brief Parse NMEA Message (only GPGGA and GPRMC)
 *
 * @param message: Pointer to cstring which contains received & parsed, single NMEA message
 * @param info: Pointer to NEO6 gps info struct to store parsed information
 *
 * @retval void
*/
void NMEA_MessageParse(char *message, struct NEO6_ParsedInfo *info)
{
        char *parsed_token;
        parsed_token = strtoke(message, ",*");

	if (!strcmp(parsed_token, "$GPGGA") || !strcmp(parsed_token, "$$GPGGA")) {
		NMEA_GPGGAParse(info);
        }

        else if (!strcmp(parsed_token, "$GPRMC") || !strcmp(parsed_token, "$$GPRMC")) {
		NMEA_GPRMCParse(info);
        }

	else {
		sprintf(info->utc_time, "00:00:00");
		sprintf(info->date, "00.00.0000");

		info->quality = 0;

		info->pos.alt = 0;
		info->pos.lat = 0;
		info->pos.lat_dir = '0';
		info->pos.lon = 0;
		info->pos.lon_dir = '0';
		return ;

	}

}


/**
 * @brief Print Parsed GPS Info
 *
 * @param gps: Pointer to NEO6 GPS configuration and received-information struct
 *
 * @retval void
*/
void NEO6_PrintInfo(NEO6 *gps)
{
	if (gps->info.quality){

		char location_data[50];
		char alt_data[50];
		char date[15];
		char time[15];

		char *datalar[5];

		sprintf(location_data,"Your Location: %f %c, %f %c\n\r", gps->info.pos.lat, gps->info.pos.lat_dir, gps->info.pos.lon, gps->info.pos.lon_dir);
		sprintf(alt_data,"Your Altitude: %fm\n\r", gps->info.pos.alt);
		sprintf(date,"Date: %s\n\r", gps->info.date);
		sprintf(time,"UTC time: %s\n\r", gps->info.utc_time);

		/*
		printf("Your Location: %f %c, %f %c\n\r", gps->info.pos.lat, gps->info.pos.lat_dir, gps->info.pos.lon, gps->info.pos.lon_dir);
		printf("Your Altitude: %fm\n\r", gps->info.pos.alt);
		printf("Date: %s\n\r", gps->info.date);
		printf("UTC time: %s\n\r", gps->info.utc_time);
		printf("---------------------------------------------\n\r");
		*/

		HAL_UART_Transmit(&huart5, location_data, strlen(location_data), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart5, alt_data, strlen(alt_data), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart5, date, strlen(date), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart5, time, strlen(time), HAL_MAX_DELAY);



	}
}


/**
 * @brief Return latitude and longtitude in one string from obtained data
 *
 * @param gps: Pointer to NEO6 GPS configuration and received-information struct
 *
 * @retval Pointer to the beginning of cstring that contains location info
 *
 * @warning returning cstirng is statically declared, so consecutive
 * 	    calls to this function will overwrite the same memory spot
 *
*/
char *NEO6_GetLocation(NEO6 *gps)
{
	static char location[30];
	if (gps->info.quality)
		sprintf(location, "%f %c, %f %c", gps->info.pos.lat, gps->info.pos.lat_dir, gps->info.pos.lon, gps->info.pos.lon_dir);

	return location;

}

/**
 * @brief return date and time obtained from NEO6 GPS module.
 * 	  format: dd.mm.yyyy hh:mm:ss UTC
 *
 * @param gps: Pointer to NEO6 GPS configuration and received-information struct
 *
 * @retval Pointer to the beginning of cstring that contains date&time info
 *
 * @warning returning cstirng is statically declared, so consecutive
 * 	    calls to this function will overwrite the same memory spot
 *
*/
char *NEO6_GetDateTime(NEO6 *gps)
{
	static char date_time[32];
	if (gps->info.quality)
		sprintf(date_time, "%s %s UTC", gps->info.date, gps->info.utc_time);

	return date_time;

}


/**
 * @brief return current altitude obtained from NEO6 GPS module
 *
 * @param gps: Pointer to NEO6 GPS configuration and received-information struct
 *
 * @retval (double) value of altitude
 *
 *
*/
double NEO6_GetAltitude(NEO6 *gps)
{
	double alt;
	if (gps->info.quality)
		alt = gps->info.pos.alt;
	else
		alt = -1;

	return alt;
}

/**
 * @brief Main User function; recevies, parses and stores useful data as: location, time, date, altitude
 *
 * @param gps: Pointer to NEO6 GPS configuration and received-information struct
 * @param uart_handler: Pointer to a UART_HandleTypeDef structure that contains
 *                      the configuration information for the specified UART module
 *
 * @retval Status Code
*/
uint8_t NEO6_Init(NEO6 *gps, UART_HandleTypeDef *uart_handler)
{
        /**
         * Check for null pointer error
        */
        if (gps == NULL || uart_handler == NULL)
                return GPS_ERR_NULL_PTR;

        /**
         * Initialize gps struct
        */
        gps->com.uart = uart_handler;
        gps->com.buffer_tail = 0;
        gps->com.message_tail = 0;

        gps->info.quality = 0;

        gps->info.pos.lat = 0;
        gps->info.pos.lat_dir = '0';

        gps->info.pos.lon = 0;
        gps->info.pos.lon_dir = '0';

        gps->info.pos.alt = 0;

	sprintf(gps->info.utc_time, "00:00:00");
	sprintf(gps->info.date, "00.00.0000");

        /**
         * Begin reception of data
        */
       return NEO6_UART_ReceiveChar(gps);
}


