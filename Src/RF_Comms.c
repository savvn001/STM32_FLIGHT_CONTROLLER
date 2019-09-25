/*
 * RF_Comms.c
 *
 *  Created on: Sep 23, 2019
 *      Author: nick_savva
 */

#include "RF_Comms.h"

#include "../Drivers/dwt_delay.h"
#include "spi.h"
#include "math.h"

//////////////////////////////////// NRF24 variables //////////////////////////////
uint64_t TxpipeAddrs = 0x11223344AA;
uint8_t RxData[32];

unsigned char AckPayload_0[32];
unsigned char AckPayload_1[32];

uint32_t packetsLostCtr = 0;

uint32_t batteryLevel = 0;
uint16_t loop_counter = 0;

uint16_t lastAvgBatteryLevel;
int avgBatteryLevel = 0;

float yaw_rx = 0;
//////////////////////////////////// IMU variables /////////////////////////

/////////////////////////////////////////////////////////////////
////////////////////// Init NRF24L01 Module /////////////////////
/////////////////////////////////////////////////////////////////
void RF_init() {

#if NRF24
	DWT_Init(); //Enable some of the MCUs special registers so we can get microsecond (us) delays
	NRF24_begin(GPIOB, nrf_CSN_PIN, nrf_CE_PIN, hspi2);
	//nrf24_DebugUART_Init(huart6);
	NRF24_enableAckPayload();
	NRF24_setAutoAck(true);
	NRF24_openReadingPipe(1, TxpipeAddrs);
	NRF24_startListening();

	printRadioSettings();
#endif

}

void RF_TxRx(uint16_t *throttle, float *p_setpoint, float *r_setpoint, float *y_setpoint, float roll, float pitch, float yaw) {

#if NRF24
	//Pack acknowledge data 0 - sent every control loop
	packAckPayData_0(roll, pitch, yaw);

	//Pack acknowledge data 1 - sent every second
	if (loop_counter == CRTL_LOOP_FREQ - 1) {
		packAckPayData_1();
		loop_counter = 0;
	} else {
		loop_counter++;
	}

	//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);

	/* Get data from receiver */
	if (NRF24_available()) {
		NRF24_read(RxData, 32);

		packetsLostCtr = 0;

		//Write the acknowledge payload back to the transmitter/controller
		if (loop_counter == CRTL_LOOP_FREQ - 1) {
			NRF24_writeAckPayload(1, AckPayload_1, 32);

		} else {
			NRF24_writeAckPayload(1, AckPayload_0, 32);

		}
		//Unpack the 32 byte payload from controller
		unpackRxData();

		//Reverse R joystick positions
		Rx_Data.R_Joystick_YPos = 	4096 - Rx_Data.R_Joystick_YPos;
		Rx_Data.R_Joystick_XPos = 	4096 - Rx_Data.R_Joystick_XPos;


		//Map throttle joystick reading to ESC range
		*throttle = map(Rx_Data.L_Joystick_YPos, 850, 3300, ESC_MIN, ESC_MAX);

		//Implement a deadzone for the bottom end of values
		if (*throttle < ESC_MIN + 200) {
			*throttle = ESC_MIN;
		}

		//Implement a deadzone for the top end of values
		if (*throttle > ESC_MAX - 200) {
			*throttle = ESC_MAX;
		}

		//Map right joystick X axis to roll set point
		*r_setpoint = map(Rx_Data.R_Joystick_XPos, 350, 3940, -MAX_ANGLE, MAX_ANGLE);
		*r_setpoint += 1;
		//Clip just in case
		if(*r_setpoint > MAX_ANGLE) *r_setpoint = (float) MAX_ANGLE;
		if(*r_setpoint < -MAX_ANGLE) *r_setpoint = (float) -MAX_ANGLE;

		//Map right joystick Y axis to roll set point
		*p_setpoint = map(Rx_Data.R_Joystick_YPos, 370, 3980, -MAX_ANGLE, MAX_ANGLE);
		*p_setpoint += 1;
		//Clip just in case
		if(*p_setpoint > MAX_ANGLE) *p_setpoint = (float) MAX_ANGLE;
		if(*p_setpoint < -MAX_ANGLE) *p_setpoint = (float) -MAX_ANGLE;


		//Map left joystick X axis to yaw set point
		#define	YAW_TURN_RATE 0.5

		yaw_rx = map(Rx_Data.L_Joystick_XPos, 260, 3900, -YAW_TURN_RATE, YAW_TURN_RATE);

		if(yaw_rx>YAW_TURN_RATE) {yaw_rx = YAW_TURN_RATE;}
		if(yaw_rx<-YAW_TURN_RATE) {yaw_rx = -YAW_TURN_RATE;}

		//make small deadzone
		if(yaw_rx > -0.3 && yaw_rx < 0.3){ yaw_rx = 0.0000000f;}

		if(yaw_rx > -YAW_TURN_RATE && yaw_rx < YAW_TURN_RATE){
			(*y_setpoint) += yaw_rx;
		}

		//Clip yaw around 360 degree circle
		if(*y_setpoint > 360) *y_setpoint = (float) 0;
		if(*y_setpoint < 0) *y_setpoint = (float) 360;

	} else {
		packetsLostCtr++;
	}

	if (packetsLostCtr > 10) {
		//lostConnection();
	}

#endif
}
// Unpack received 32 byte payload from transmitter, see documentation for specification details
void unpackRxData() {

	Rx_Data.L_Joystick_XPos = (RxData[0] & 0xFF) | (RxData[1] << 8);
	Rx_Data.L_Joystick_YPos = (RxData[2] & 0xFF) | (RxData[3] << 8);
	Rx_Data.R_Joystick_XPos = (RxData[4] & 0xFF) | (RxData[5] << 8);
	Rx_Data.R_Joystick_YPos = (RxData[6] & 0xFF) | (RxData[7] << 8);

	Rx_Data.airmode = (RxData[8] >> 0) & 1;

	Rx_Data.kill_rx = (RxData[8] >> 1) & 1;

	if (Rx_Data.kill_rx) {
		//kill();
	}

	//Unpack PID data
	//	uint16_t roll_p_rx = (RxData[9] & 0xFF) | (RxData[10] << 8);
	//	uint16_t roll_i_rx = (RxData[11] & 0xFF) | (RxData[12] << 8);
	//	uint16_t roll_d_rx = (RxData[13] & 0xFF) | (RxData[14] << 8);

	//	//Remap
	//	pitch_p_gain = (float) roll_p_rx / 100;
	//	pitch_i_gain = (float) roll_i_rx / 100;
	//	pitch_d_gain = (float) roll_d_rx / 100;

}

//	Pack acknowledge payload data 0 - sent every control loop, which will be sent back to controller once drone has successfully
//	received a payload from it
void packAckPayData_0(float roll, float pitch, float yaw) {

	//ID for packet 0
	AckPayload_0[0] = 0x03;

	//Next 2 bytes = Battery level
	AckPayload_0[1] = batteryLevel;
	AckPayload_0[2] = batteryLevel >> 8;

	//Next 4 bytes = IMU Roll
	int16_t roll_tx = round(roll * 100);
	AckPayload_0[3] = roll_tx;
	AckPayload_0[4] = roll_tx >> 8;

	//Next  4 bytes = IMU Pitch
	int16_t pitch_tx = round(pitch * 100);
	AckPayload_0[5] = pitch_tx;
	AckPayload_0[6] = pitch_tx >> 8;

	//Next  4 bytes = IMU Yaw
	int16_t yaw_tx = round(yaw * 100);
	AckPayload_0[7] = yaw_tx;
	AckPayload_0[8] = yaw_tx >> 8;

}

//	Pack acknowledge payload data 1 - sent every second, contains GPS data
void packAckPayData_1() {

	//ID for packet 1
	AckPayload_1[0] = 0xFF;

	getGPSData(AckPayload_1);

}


float map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

