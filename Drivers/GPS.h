/*
 * 	Driver for interfacing with a NEO6M GPS module and parsing the received NMEA data.
 *
 * 	The UART_timeout function gets called by the UART6 Idle line detection interrupt (implemented by the hacks made to the STM UART driver).
 * 	This is when a full set of sentences (8 in my case) ha been received and stored in the GPS_RX_Buffer.
 * 	The buffer is of course, bigger than what all of the sentences might be as we don't know how long the all of the sentences
 *  combined are beforehand.
 *	Each sentence is extracted from the buffer one by one, then parsed by the library which parses a single sentence.
 *
 * 	The sentence extraction code was obtained from https://github.com/kosma/minmea
 *
 *	Nicholas Savva, 2019
 *
 */



#include "stm32f4xx_hal.h"
#include "usart.h"
#include "minnmea/minmea.h"

#define GPS_BUFFERSIZE 1000

//Number of NMEA sentences ouput by GPS module, by default mine is 8
#define NO_OF_SENTENCES 8

#define INDENT_SPACES "  "
uint8_t GPS_RX_Buffer[GPS_BUFFERSIZE];

bool first = 1;
char line[MINMEA_MAX_LENGTH];

void GPS_parse();
void parse_sentence();


struct GPS_{

	uint8_t Day;
	uint8_t Month;
	uint8_t Year;

	uint8_t Hours;
	uint8_t Minutes;
	uint8_t Seconds;

	float Longitude;
	float Latitude;
	float Speed;
	float Altitude;

	uint8_t sattelite_no;
	uint8_t fix_quality;

};


struct GPS_ GPS;


void GPS_init() {

	for (int i = 0; i < GPS_BUFFERSIZE; ++i) {
		GPS_RX_Buffer[i] = 0;
	}

	HAL_UART_Receive_DMA(&huart6, GPS_RX_Buffer, GPS_BUFFERSIZE);


}

void UART_timeout() {


	if (first) {
		first = 0;
	} else {
		//Parse the NMEA message
		GPS_parse();
	}


	//Reset buffer to all 0s
	for (int i = 0; i < GPS_BUFFERSIZE; ++i) {
		GPS_RX_Buffer[i] = 0;
	}

	//Begin receiving again
	HAL_UART_Receive_DMA(&huart6, GPS_RX_Buffer, GPS_BUFFERSIZE);


}

void GPS_parse() {

	//The RX buffer index
	uint16_t i = 0;
	uint16_t last_i = 0;

	//Get the sentences out of the buffer so we can then parse them using the library
	for (int y = 0; y < 8; y++) {

		//Init all elements of the current line to be stored to 0
		for (int z = 0; z < MINMEA_MAX_LENGTH; z++) {
			line[z] = 0;
		}

		//Have to manually scan across until we reach end of each sentence as don't know how
		//long each one is beforehand

		last_i = i;

		//First element of each sentence should be a $
		if (GPS_RX_Buffer[i] == 0x24) {

			//Last element of line should be a new line feed symbol
			while (GPS_RX_Buffer[i] != 0x0A) {
				//This index will be how long the current sentence is
				i++;
			}
		}

		//Store this sentence in another buffer that is JUST that sentence only
		for (int j = 0; j <= (i-last_i); j++) {
			line[j] = GPS_RX_Buffer[j+last_i];
		}

		//Parse this line
		parse_sentence();

		i++;
	}
}

//From https://github.com/kosma/minmea
void parse_sentence() {

	//char line[MINMEA_MAX_LENGTH];

	//while (fgets(line, MINMEA_MAX_LENGTH, stdin) != NULL) {
	//printf("%s", line);
	switch (minmea_sentence_id(line, false)) {
	case MINMEA_SENTENCE_RMC: {
		struct minmea_sentence_rmc frame;
		if (minmea_parse_rmc(&frame, line)) {
			printf(
					INDENT_SPACES "$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
					frame.latitude.value, frame.latitude.scale,
					frame.longitude.value, frame.longitude.scale,
					frame.speed.value, frame.speed.scale);
			printf(
					INDENT_SPACES "$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
					minmea_rescale(&frame.latitude, 1000),
					minmea_rescale(&frame.longitude, 1000),
					minmea_rescale(&frame.speed, 1000));
			//			printf(
			//					INDENT_SPACES "$xxRMC floating point degree coordinates and speed: (%f,%f) %f\n",
			//					minmea_tocoord(&frame.latitude),
			//					minmea_tocoord(&frame.longitude),
			//					minmea_tofloat(&frame.speed));

			GPS.Longitude = minmea_tocoord(&frame.longitude);
			GPS.Latitude = minmea_tocoord(&frame.latitude);

			GPS.Day = frame.date.day;
			GPS.Month = frame.date.month;
			GPS.Year = frame.date.year;

		} else {
			printf(INDENT_SPACES "$xxRMC sentence is not parsed\n");
		}
	}
	break;

	case MINMEA_SENTENCE_GGA: {
		struct minmea_sentence_gga frame;
		if (minmea_parse_gga(&frame, line)) {

			GPS.Hours = frame.time.hours;
			GPS.Minutes = frame.time.minutes;
			GPS.Seconds = frame.time.seconds;

			GPS.sattelite_no = frame.satellites_tracked;
			GPS.fix_quality = frame.fix_quality;

			GPS.Altitude = minmea_tofloat(&frame.altitude);


			//			printf(INDENT_SPACES "$xxGGA: fix quality: %d\n",
			//					frame.fix_quality);
		} else {
			printf(INDENT_SPACES "$xxGGA sentence is not parsed\n");
		}
	}
	break;

	case MINMEA_SENTENCE_GST: {
		struct minmea_sentence_gst frame;
		if (minmea_parse_gst(&frame, line)) {
			printf(
					INDENT_SPACES "$xxGST: raw latitude,longitude and altitude error deviation: (%d/%d,%d/%d,%d/%d)\n",
					frame.latitude_error_deviation.value,
					frame.latitude_error_deviation.scale,
					frame.longitude_error_deviation.value,
					frame.longitude_error_deviation.scale,
					frame.altitude_error_deviation.value,
					frame.altitude_error_deviation.scale);
			printf(
					INDENT_SPACES "$xxGST fixed point latitude,longitude and altitude error deviation"
					" scaled to one decimal place: (%d,%d,%d)\n",
					minmea_rescale(&frame.latitude_error_deviation, 10),
					minmea_rescale(&frame.longitude_error_deviation, 10),
					minmea_rescale(&frame.altitude_error_deviation, 10));
			printf(
					INDENT_SPACES "$xxGST floating point degree latitude, longitude and altitude error deviation: (%f,%f,%f)",
					minmea_tofloat(&frame.latitude_error_deviation),
					minmea_tofloat(&frame.longitude_error_deviation),
					minmea_tofloat(&frame.altitude_error_deviation));
		} else {
			printf(INDENT_SPACES "$xxGST sentence is not parsed\n");
		}
	}
	break;

	case MINMEA_SENTENCE_GSV: {
		struct minmea_sentence_gsv frame;
		if (minmea_parse_gsv(&frame, line)) {
			printf(INDENT_SPACES "$xxGSV: message %d of %d\n", frame.msg_nr,
					frame.total_msgs);
			printf(INDENT_SPACES "$xxGSV: sattelites in view: %d\n",
					frame.total_sats);
			for (int i = 0; i < 4; i++)
				printf(
						INDENT_SPACES "$xxGSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
						frame.sats[i].nr, frame.sats[i].elevation,
						frame.sats[i].azimuth, frame.sats[i].snr);
		} else {
			printf(INDENT_SPACES "$xxGSV sentence is not parsed\n");
		}
	}
	break;

	case MINMEA_SENTENCE_VTG: {
		struct minmea_sentence_vtg frame;
		if (minmea_parse_vtg(&frame, line)) {

			GPS.Speed = minmea_tofloat(&frame.speed_kph);


			printf(INDENT_SPACES "$xxVTG: true track degrees = %f\n",
					minmea_tofloat(&frame.true_track_degrees));
			printf(INDENT_SPACES "        magnetic track degrees = %f\n",
					minmea_tofloat(&frame.magnetic_track_degrees));
			printf(INDENT_SPACES "        speed knots = %f\n",
					minmea_tofloat(&frame.speed_knots));
			printf(INDENT_SPACES "        speed kph = %f\n",
					minmea_tofloat(&frame.speed_kph));
		} else {
			printf(INDENT_SPACES "$xxVTG sentence is not parsed\n");
		}
	}
	break;

	case MINMEA_SENTENCE_ZDA: {
		struct minmea_sentence_zda frame;
		if (minmea_parse_zda(&frame, line)) {
			printf(
					INDENT_SPACES "$xxZDA: %d:%d:%d %02d.%02d.%d UTC%+03d:%02d\n",
					frame.time.hours, frame.time.minutes, frame.time.seconds,
					frame.date.day, frame.date.month, frame.date.year,
					frame.hour_offset, frame.minute_offset);
		} else {
			printf(INDENT_SPACES "$xxZDA sentence is not parsed\n");
		}

		//		gps_hours = frame.time.hours;
		//		gps_minutes = frame.time.minutes;
		//		gps_seconds = frame.time.seconds;
	}
	break;

	case MINMEA_INVALID: {
		printf(INDENT_SPACES "$xxxxx sentence is not valid\n");
	}
	break;

	default: {
		printf(INDENT_SPACES "$xxxxx sentence is not parsed\n");
	}
	break;
	}
	//}

}

void getGPSData(uint8_t *ack_payload_1){

	//Goto element [1]
	ack_payload_1++;

	//[1] = no of sattelites tracked
	*ack_payload_1 = GPS.sattelite_no;
	ack_payload_1++;
	//[2] = Fix quality
	*ack_payload_1 = GPS.fix_quality;
	ack_payload_1++;
	//[3] = Day
	*ack_payload_1 = GPS.Day;
	ack_payload_1++;
	//[4] = Month
	*ack_payload_1 = GPS.Month;
	ack_payload_1++;
	//[5] = Year
	*ack_payload_1 = GPS.Year;
	ack_payload_1++;
	//[6] = Hour
	*ack_payload_1 = GPS.Hours;
	ack_payload_1++;
	//[7] = Minutes
	*ack_payload_1 = GPS.Minutes;
	ack_payload_1++;
	//[8] = Hours
	*ack_payload_1 = GPS.Hours;
	ack_payload_1++;

	//[9] - [10] = GPS speed in KM/H
	int16_t GPS_speed_tx = round(GPS.Speed * 100);
	*ack_payload_1 = GPS_speed_tx;
	ack_payload_1++;
	*ack_payload_1 = GPS_speed_tx >> 8;
	ack_payload_1++;

	//[11] - [14] = Longitude
	unsigned char temp[sizeof(float)];
	memcpy(temp, &GPS.Longitude, sizeof(GPS.Longitude));

	*ack_payload_1 = temp[0];
	ack_payload_1++;
	*ack_payload_1 = temp[1];
	ack_payload_1++;
	*ack_payload_1 = temp[2];
	ack_payload_1++;
	*ack_payload_1 = temp[3];
	ack_payload_1++;

	//[15] - [18] = Longitude
	memcpy(temp, &GPS.Latitude, sizeof(GPS.Latitude));

	*ack_payload_1 = temp[0];
	ack_payload_1++;
	*ack_payload_1 = temp[1];
	ack_payload_1++;
	*ack_payload_1 = temp[2];
	ack_payload_1++;
	*ack_payload_1 = temp[3];
	ack_payload_1++;


	//[19] - [22] = Longitude
	memcpy(temp, &GPS.Altitude, sizeof(GPS.Altitude));

	*ack_payload_1 = temp[0];
	ack_payload_1++;
	*ack_payload_1 = temp[1];
	ack_payload_1++;
	*ack_payload_1 = temp[2];
	ack_payload_1++;
	*ack_payload_1 = temp[3];
	ack_payload_1++;


}
