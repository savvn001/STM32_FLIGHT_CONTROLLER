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



#include <stdio.h>
#include <stdbool.h>
#include "RF_Comms.h"



////////////////////////// "Public" functions ////////////////////////
void CL_init();
void CL_main();


//////////////////////// "Private" functions ////////////////////////


//These 4 functions set the PWM duty cycles
void PWM1_Set(uint16_t value);
void PWM2_Set(uint16_t value);
void PWM3_Set(uint16_t value);
void PWM4_Set(uint16_t value);

void lostConnection();
void resetNRF24();

#endif /* CONTROLLOOP_H_ */
