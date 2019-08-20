/*
 * PID.h
 *
 *  Created on: Jun 19, 2019
 *      Author: nick_savva
 */

#ifndef PID_H_
#define PID_H_


//PID gain values
float roll_p_gain; //9
float roll_i_gain; //0.8
float roll_d_gain; //1.0

float pid_calculate_roll(float IMU_roll_value, int timer_value, float roll_setpoint);
float pid_calculate_pitch(float IMU_pitch_value, int timer_value, float pitch_setpoint);
float pid_calculate_yaw(float IMU_yaw_value, int timer_value, float yaw_setpoint);
void reset_pid_roll();
void reset_pid_pitch();
void reset_pid_yaw();

#endif /* PID_H_ */
