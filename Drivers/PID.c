/*
 * PID.cpp
 *
 *  Created on: Jun 19, 2019
 *      Author: nick_savva
 */

#include "PID.h"

//Roll Axis PID control variables

//The receiver value is the desired orientation
float receiver = 0;
//PID gain values
float roll_p_gain = 1;
float roll_i_gain = 0.;
float roll_d_gain = 0;

float roll_setpoint = 0;
float roll_p_gain_max = 400; //Set a maximum value to prevent extreme outputs
float roll_error = 0;
float roll_i_mem = 0;
float roll_p;
float roll_i;
float roll_d;
float roll_output = 0;
float roll_last_d_error = 0;

int now = 0;
int last_update = 0;

float pid_calculate_roll(float IMU_roll_value, int timer_value) {

	//Roll calculations

	//Calculate error
	roll_error = IMU_roll_value - roll_setpoint;

	//Proportional component
	roll_p = roll_p_gain*roll_error;


	//Derivative component
	now = timer_value;
	//Elaspsed time in seconds from counter value = (ticks) * (1/ (timer clk freq / prescaler value))
	float elapsed_time  = (float) ((now - last_update) * (1 / (84000000.0f / 65535.0f)));
	roll_d = roll_d_gain * ( (roll_error - roll_last_d_error) / elapsed_time);
	last_update = now;
	roll_last_d_error = roll_error;


	//Integral, error over time
	roll_i += (roll_i_gain*roll_error);

//	//Limit max output to prevent extreme swings
//	if (roll_i_mem > roll_p_gain_max)
//		roll_i_mem = roll_p_gain_max;
//	else if (roll_i_mem < roll_p_gain_max * -1)
//		roll_i_mem = roll_p_gain_max * -1;

	//PID together
	roll_output = roll_p + roll_i + roll_d;

//	if (roll_output > roll_p_gain_max)
//		roll_output = roll_p_gain_max;
//	else if (roll_output < roll_p_gain_max * -1)
//		roll_p_gain_max = roll_p_gain_max * -1;


	return roll_output;

}

//Pitch Axis PID control variables

//PID gain values
float pitch_p_gain = 1;
float pitch_i_gain = 1;
float pitch_d_gain = 1;

float pitch_setpoint = 0;
float pitch_p_gain_max = 400; //Set a maximum value to prevent extreme outputs
float pitch_error = 0;
float pitch_i_mem = 0;
float pitch_p;
float pitch_i;
float pitch_d;
float pitch_output = 0;
float pitch_last_d_error = 0;

float pid_calculate_pitch(float IMU_pitch_value, int timer_value) {

	//Pitch calculations

	//Calculate error
	pitch_error = IMU_pitch_value - pitch_setpoint;

	//Proportional component
	pitch_p = pitch_p_gain*pitch_error;


	//Derivative component
	now = timer_value;
	float elapsed_time  = (float) ((now - last_update) * (1 / (84E6 / 2000.0f)));
	pitch_d = pitch_d_gain * ( (pitch_error - pitch_last_d_error) / elapsed_time);
	last_update = now;
	pitch_last_d_error = pitch_error;


	//Integral, error over time
	pitch_i += (pitch_i_gain*pitch_error);

	//Limit max output to prevent extreme swings
	if (pitch_i_mem > pitch_p_gain_max)
		pitch_i_mem = pitch_p_gain_max;
	else if (pitch_i_mem < pitch_p_gain_max * -1)
		pitch_i_mem = pitch_p_gain_max * -1;

	//PID together
	pitch_output = pitch_p + pitch_i + pitch_d;

	if (pitch_output > pitch_p_gain_max)
		pitch_output = pitch_p_gain_max;
	else if (pitch_output < pitch_p_gain_max * -1)
		pitch_p_gain_max = pitch_p_gain_max * -1;


	return pitch_output;

}
