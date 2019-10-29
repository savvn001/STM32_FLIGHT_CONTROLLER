/*
 * controlLoop.h
 *
 *  Created on: Sep 23, 2019
 *      Author: nick_savva
 */

#ifndef CONTROLLOOP_H_
#define CONTROLLOOP_H_


//How many of same pulse to send before updating with new value
#define PULSE_DIV 4


//Value of ADC reading that corresponds to around 11.5V - will shut off to protect battery (3S)
#define ADC_BATTERY_SHUTOFF 3545

#define ESC_MIN 1250
#define ESC_MAX 2500

#include <stdio.h>
#include <stdbool.h>

extern struct RxTxVars_ RxTxVarsMain;



////////////////////////// "Public" functions ////////////////////////
void CL_init();
void CL_main(bool airmode, uint16_t throttle, float pitch_setpoint, float roll_setpoint, float yaw_setpoint, float *roll, float *pitch, float *yaw);


//////////////////////// "Private" functions ////////////////////////


//These 4 functions set the PWM duty cycles
void PWM1_Set(uint16_t value);
void PWM2_Set(uint16_t value);
void PWM3_Set(uint16_t value);
void PWM4_Set(uint16_t value);

void lostConnection();
void resetNRF24();

#endif /* CONTROLLOOP_H_ */
