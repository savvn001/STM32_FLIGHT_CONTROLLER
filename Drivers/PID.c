/*
 * PID.cpp
 *
 *  Created on: Jun 19, 2019
 *      Author: nick_savva
 */

#include "PID.h"

#define TIMER_CLK_FREQ 100000000.0f
/* Time of loop control loop measured using an oscilloscope gives about a 505.25Hz
 * rate, so delta_t is the time period of this and used in the PID calculations as
 * the time difference between each sample point */
#define delta_t 0.001978239367


/********************** Pitch Axis PID control variables *****************************/

//PID gain values
float pitch_p_gain = 9;
float pitch_i_gain = 0.8;
float pitch_d_gain = 1.0;

float pitch_setpoint = 0;
float pitch_output_temp = 0;
float pitch_error = 0;
float pitch_p;
float pitch_i;
float pitch_d;
float pitch_output = 0;
float pitch_last_d_error = 0;

int pitch_now = 0;
int pitch_last_update = 0;
float pitch_elapsed_time = 0;

int pitch_pid_clip = 1250;

/** Pitch PID Calculation **/
float pid_calculate_pitch(float IMU_pitch_value, int timer_value) {

	//Calculate error
	pitch_error = IMU_pitch_value - pitch_setpoint;

	//Proportional component
	pitch_p = pitch_p_gain * pitch_error;

	//Integral
	pitch_i += (pitch_i_gain * pitch_error * delta_t);


	//Derivative component

	pitch_d = pitch_d_gain * ( (pitch_error - pitch_last_d_error) / delta_t);
	pitch_last_d_error = pitch_error;

	//PID together
	pitch_output = pitch_p + pitch_i + pitch_d;

	//Clip PID output in event of extreme swings
	if (pitch_output < -pitch_pid_clip) {
		pitch_output = -pitch_pid_clip;
	} else if (pitch_output > pitch_pid_clip) {
		pitch_output = pitch_pid_clip;
	}

	return pitch_output;
}

void reset_pid_pitch(){

	pitch_p = 0;
	pitch_i = 0;
	pitch_d = 0;
	pitch_output = 0;

}

