/*
 * PID.cpp
 *
 *  Created on: Jun 19, 2019
 *      Author: nick_savva
 */

#include "PID.h"

#define TIMER_CLK_FREQ 100000000.0f



//Roll Axis PID control variables

//The receiver value is the desired orientation
float receiver = 0;
//PID gain values
float roll_p_gain = 20;
float roll_i_gain = 0;
float roll_d_gain = 0;

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

//Pitch Axis PID control variables

//PID gain values
float pitch_p_gain = 15; //2.8
float pitch_i_gain = 0.05;//0.002; //0.002
float pitch_d_gain = 0; //0.1

float pitch_setpoint = 0;
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

float pid_calculate_pitch(float IMU_pitch_value, int timer_value) {

	//pitch calculations

	//Calculate error
	pitch_error = IMU_pitch_value - pitch_setpoint;

	//Proportional component
	pitch_p = pitch_p_gain * pitch_error;

	//Integral
	pitch_i += (pitch_i_gain * pitch_error);

	/*//Clip i component?
		if (pitch_i < -800) {
			pitch_i = -800;
		} else if (pitch_i > 800) {
			pitch_i = 800;
		}*/

	//Derivative component
	pitch_now = timer_value;

	if (pitch_now - pitch_last_update < 0) {
		//Take time difference taking into account reset of timer
		//Formula for getting timer count into seconds = COUNT * (1/TIMER_CLK)*PRESCALER
		pitch_elapsed_time = (float) (((65535 - pitch_last_update) + pitch_now)* (1 / (100000000.0f / 2000.0f)));

	} else {
		//Otherwise normally the count difference will be positive
		pitch_elapsed_time = (float) ((pitch_now - pitch_last_update) * (1 / (100000000.0f / 2000.0f))); // set integration time by time elapsed since last filter update
	}

	pitch_d = pitch_d_gain
			* ((pitch_error - pitch_last_d_error) / pitch_elapsed_time);
	pitch_last_update = pitch_now;
	pitch_last_d_error = pitch_error;

	//PID together
	pitch_output = pitch_p + pitch_i + pitch_d;

	//Clip PID output in event of extreme swings
	if (pitch_output < -2000) {
		pitch_output = -2000;
	} else if (pitch_output > 2000) {
		pitch_output = 2000;
	}

	return pitch_output;
}
