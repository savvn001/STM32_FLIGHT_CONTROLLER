#include "stm32f4xx_hal.h"
#include "usart.h"
#define GPS_BUFFERSIZE 600
uint8_t GPS_RX_Buffer[GPS_BUFFERSIZE];

bool first = 1;

void GPS_init(){


	HAL_UART_Receive_DMA(&huart6, GPS_RX_Buffer, GPS_BUFFERSIZE);

	for (int i = 0; i < GPS_BUFFERSIZE; ++i) {
		GPS_RX_Buffer[i] = 0;
	}

}

void GPS_parse(){

	HAL_UART_Receive_DMA(&huart6, GPS_RX_Buffer, GPS_BUFFERSIZE);

	if(first){
		first = 0;
	}
	else{

		//Parse the NMEA message



	}

	for (int i = 0; i < GPS_BUFFERSIZE; ++i) {
		GPS_RX_Buffer[i] = 0;
	}


}
