/*
 * GPS.h
 *
 *  Created on: 6 Eyl 2022
 *      Author: FURKAN
 */

#ifndef SENSOR_DRIVERS_INC_GPS_H_
#define SENSOR_DRIVERS_INC_GPS_H_


#define GPS_MESSAGE_SIZE 90 // Maximum possible size of NMEA message
#define GPS_BUFFER_SIZE 7 // Maximum number of messages to receive (< 256)

// Status Codes
//   HAL_OK       = 0x00U,
//   HAL_ERROR    = 0x01U,
//   HAL_BUSY     = 0x02U,
//   HAL_TIMEOUT  = 0x03U
#define GPS_OK 0x04U
#define GPS_ERR_NULL_PTR 0x05U
#define GPS_BUF_FULL 0x06U
#define GPS_MSG_CPLT 0x07U
#define GPS_CHR_RECEIVED 0x08U
#define GPS_MESSAGE_INVALID 0x09U

// ****************************************************
//          Data Structures                           *
// ****************************************************

/**
 * Position Data, latitude, longtitude, altitude
 *
*/
struct position_data
{
        // latitude of location in degrees and fractions of degrees (18.1231)
        double lat;
        // direction of latitude value e.g. N or S
        char lat_dir;

        // longtitude of location in degrees and fractions of degrees
        double lon;
        // direction of longtitude value e.g. E or W
        char lon_dir;

        // altitude of the object
        double alt;
};



/**
 * Data from NEO6m GPS module after parsing
 *
*/
struct NEO6_ParsedInfo
{
        /**
         * Position data obtained and parsed from NEO6 GPS module
         *
         * Includes:
         *      latitude (val and dir)
         *      longtitude (val and dir)
         *      altitude
        */
        struct position_data pos;

        /**
         * UTC time of FIX
         *
         * Format: hh:mm:ss (UTC)
        */
        char utc_time[14];

        /**
         * Date of FIX
         *
         * Format: dd.mm.yyyy
        */
        char date[14];

        /**
         * Quality of fix
         *
         * 0 = Invalid, 1 = GPS fix, 2 = DGPS fix, 3 = PPS fix, 4 = Real Time Kinematic, etc.
        */
        uint8_t quality;
};



/**
 * Struct for communication configurations
 *
*/
struct NEO6_ComConf {
        /**
         * HAL UART Communication handler typedef
         *
        */
        UART_HandleTypeDef *uart;

        /**
         * Message buffer to temporarily hold some amount of
         *  received NMEA messages before parsing
         *
         * Mainly used to find average location
         *
        */
        char messages_buffer[(uint8_t)GPS_BUFFER_SIZE][(uint8_t)GPS_MESSAGE_SIZE];

        /**
        * Buffer tail is indicating the index of next free part of message_buffer
        * -1 if message_buffer is full
        */
        int8_t buffer_tail;

        /**
         * Message tail is indicating the index of next free place to save received char
         *
        */
       int8_t message_tail;
};



/**
 * Main user struct for handling NEO6 GPS device
 *
*/
typedef struct {
        struct NEO6_ParsedInfo info;
        struct NEO6_ComConf com;

}NEO6;



// ****************************************************
//          Function Prototypes                       *
// ****************************************************

uint8_t NEO6_Init(NEO6 *gps, UART_HandleTypeDef *uart_handler);
uint8_t NEO6_UART_ReceiveChar(NEO6 *gps);

char *NEO6_GetLocation(NEO6 *gps);
char *NEO6_GetDateTime(NEO6 *gps);
double NEO6_GetAltitude(NEO6 *gps);
void NEO6_PrintInfo(NEO6 *gps);

void NMEA_MessageParse(char *message, struct NEO6_ParsedInfo *info);


#endif /* SENSOR_DRIVERS_INC_GPS_H_ */
