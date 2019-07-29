/*
 * PID.h
 *
 *  Created on: Jun 19, 2019
 *      Author: nick_savva
 */

#ifndef PID_H_
#define PID_H_


float pid_calculate_roll(float IMU_roll_value, int timer_value, float roll_setpoint);
float pid_calculate_pitch(float IMU_pitch_value, int timer_value, float pitch_setpoint);
float pid_calculate_yaw(float IMU_yaw_value, int timer_value, float yaw_setpoint);
void reset_pid_roll();
void reset_pid_pitch();
void reset_pid_yaw();

#endif /* PID_H_ */
