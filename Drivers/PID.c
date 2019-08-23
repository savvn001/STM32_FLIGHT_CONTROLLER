/*
 * PID.cpp
 *
 *  Created on: Jun 19, 2019
 *      Author: nick_savva
 *
 *
 *     Driver for calculating the desired PID response for roll, pitch & yaw axes based off of IMU readings,
 *     and setpoints from controller.
 */

#include "PID.h"

#define TIMER_CLK_FREQ 100000000.0f
/* Time of loop control loop configured as 500Hz, so
 * delta_t is the time period of this and used in the PID calculations as
 * the time difference between each sample point */
#define delta_t 0.001970831691f

//PID gain values
float roll_p_gain = 2.5; //9
float roll_i_gain = 2.0; //0.8
float roll_d_gain = 0.42; //1.0





/********************** roll Axis PID control variables *****************************/



float roll_output_temp = 0;
float roll_error = 0;
float roll_p;
float roll_i;
float roll_d;

float roll_output = 0;
float roll_last_d_error = 0;

int roll_now = 0;
int roll_last_update = 0;
float roll_elapsed_time = 0;

int roll_pid_clip = 1250;

/** roll PID Calculation **/
float pid_calculate_roll(float IMU_roll_value, int timer_value, float roll_setpoint) {

	//Calculate error
	roll_error = IMU_roll_value - roll_setpoint;

	//Proportional component
	roll_p = roll_p_gain * roll_error;

	//Integral
	roll_i += (roll_i_gain * roll_error * delta_t);

	//Derivative component
	roll_d = roll_d_gain * ( (roll_error - roll_last_d_error) / delta_t);
	roll_last_d_error = roll_error;

	//PID together
	roll_output = roll_p + roll_i + roll_d;

	//Clip PID output in event of extreme swings
	if (roll_output < -roll_pid_clip) {
		roll_output = -roll_pid_clip;
	} else if (roll_output > roll_pid_clip) {
		roll_output = roll_pid_clip;
	}

	return roll_output;
}

void reset_pid_roll(){

	roll_p = 0;
	roll_i = 0;
	roll_d = 0;
	roll_output = 0;

}




/********************** Pitch Axis PID control variables *****************************/


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
float pid_calculate_pitch(float IMU_pitch_value, int timer_value, float pitch_setpoint) {

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



/********************** yaw Axis PID control variables *****************************/

//PID gain values
float yaw_p_gain = 9;
float yaw_i_gain = 0.8;
float yaw_d_gain = 1.0;

float yaw_output_temp = 0;
float yaw_error = 0;
float yaw_p;
float yaw_i;
float yaw_d;
float yaw_output = 0;
float yaw_last_d_error = 0;

int yaw_now = 0;
int yaw_last_update = 0;
float yaw_elapsed_time = 0;

int yaw_pid_clip = 1250;

/** yaw PID Calculation **/
float pid_calculate_yaw(float IMU_yaw_value, int timer_value, float yaw_setpoint) {

	//Calculate error
	yaw_error = IMU_yaw_value - yaw_setpoint;

	//Proportional component
	yaw_p = yaw_p_gain * yaw_error;

	//Integral
	yaw_i += (yaw_i_gain * yaw_error * delta_t);

	//Derivative component
	yaw_d = yaw_d_gain * ( (yaw_error - yaw_last_d_error) / delta_t);
	yaw_last_d_error = yaw_error;

	//PID together
	yaw_output = yaw_p + yaw_i + yaw_d;

	//Clip PID output in event of extreme swings
	if (yaw_output < -yaw_pid_clip) {
		yaw_output = -yaw_pid_clip;
	} else if (yaw_output > yaw_pid_clip) {
		yaw_output = yaw_pid_clip;
	}

	return yaw_output;
}

void reset_pid_yaw(){

	yaw_p = 0;
	yaw_i = 0;
	yaw_d = 0;
	yaw_output = 0;

}


