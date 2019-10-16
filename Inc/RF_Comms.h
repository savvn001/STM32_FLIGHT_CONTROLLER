/*
 * RF_Comms.h
 *
 *  Created on: Sep 23, 2019
 *      Author: nick_savva
 */

#ifndef RF_COMMS_H_
#define RF_COMMS_H_

#include <stdio.h>

#include "../Drivers/MY_NRF24.h"



//1 if NRF24 module connected
#define NRF24 1
#define CRTL_LOOP_FREQ 500

//Min and max counter load value for timer4, which handles PWM generation. ESC_MIN = 125us pulse
//and ESC_MAX = 250us pulse (OneShot125 protocol)
#define ESC_MIN 1250
#define ESC_MAX 2500

//Max angle on all axis
#define MAX_ANGLE 20

void RF_init();
uint16_t RF_TxRx(uint16_t *throttle, int *p_setpoint, int *r_setpoint, float *y_setpoint, float roll, float pitch, float yaw);
void unpackRxData();
void packAckPayData_0(float roll, float pitch, float yaw);
void packAckPayData_1();
float map(float x, float in_min, float in_max, float out_min, float out_max);

struct RX_Data_Str{

	//These hold the received joystick positions from the transmitter, left and right respectively
	int16_t L_Joystick_XPos;
	int16_t L_Joystick_YPos;
	int16_t R_Joystick_XPos;
	int16_t R_Joystick_YPos;

	char airmode;
	char kill_rx;
};

struct RX_Data_Str Rx_Data;
#endif /* RF_COMMS_H_ */
