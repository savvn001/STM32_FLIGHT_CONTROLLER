/*
 * PID.cpp
 *
 *  Created on: Jun 19, 2019
 *      Author: nick_savva
 */

#include "PID.h"

#define TIMER_CLK_FREQ 100000000.0f
#define delta_t 0.001978239367 //ESCs update at 505.5hz, time period of this


//Roll Axis PID control variables

//The receiver value is the desired orientation
float receiver = 0;
//PID gain values
float roll_p_gain = 13.2;
float roll_i_gain = 20.2;
float roll_d_gain = 3.586;

float roll_setpoint = 0;
float roll_p_gain_max = 400; //Set a maximum value to prevent extreme outputs
float roll_error = 0;
float roll_p;
float roll_i;
float roll_d;
float roll_output = 0;
float roll_last_d_error = 0;

int roll_now = 0;
int roll_last_update = 0;

float pid_calculate_roll(float IMU_roll_value, int timer_value) {

	//Roll calculations

	//Calculate error
	roll_error = IMU_roll_value - roll_setpoint;

	//Proportional component
	roll_p = roll_p_gain * roll_error;

	//Integral
	roll_i += (roll_i_gain * roll_error);

	//Derivative component
	roll_now = timer_value;
	//Elaspsed time in seconds from counter value = (ticks) * (1/ (timer clk freq / prescaler value))
	float elapsed_time = (float) ((roll_now - roll_last_update)
			* (1 / (TIMER_CLK_FREQ / 2000.0f)));
	roll_d = roll_d_gain * ((roll_error - roll_last_d_error) / elapsed_time);
	roll_last_update = roll_now;
	roll_last_d_error = roll_error;

	//PID together
	roll_output = roll_p + roll_i + roll_d;

	//Clip PID output in event of extreme swings
	//Only 10 degrees for initial test so 400 is fine
	if (roll_output < -400) {
		roll_output = -400;
	} else if (roll_output > 400) {
		roll_output = 400;
	}

	return roll_output;

}

/********************** Pitch Axis PID control variables *****************************/

//PID gain values
float pitch_p_gain = 8.5; //12 causes oscillation
float pitch_i_gain = 0.5;
float pitch_d_gain = 0.8;

float pitch_setpoint = 0;
float pitch_output_temp = 0;
float pitch_p_gain_max = 400; //Set a maximum value to prevent extreme outputs
float pitch_error = 0;
float pitch_i_mem = 0;
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

	//Only want it to act close to centre
	//if(pitch_error < 10 && pitch_error > -10){
	pitch_i += (pitch_i_gain * pitch_error * delta_t);
	//}

	//Derivative component

	//Now actually work out d component
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


