/*
 * GPS.h
 *
 *  Created on: Jul 30, 2019
 *      Author: nick_savva
 */

#ifndef GPS_H_
#define GPS_H_

#include <stdint.h>

//GPS module data struct
typedef struct{

	float speed;
	float longitude;
	float latitude;
	float altitude;
	float course;
	char satellites;
	char day;
	char month;
	char year;
	char second;
	char minute;
	char hour;

} GPS_typedef;

GPS_typedef GPS;


uint8_t GPS_RX_Buffer[600];



void parse_GPS_data();

#endif /* GPS_H_ */
