/*
 * PID.h
 *
 *  Created on: Jun 19, 2019
 *      Author: nick_savva
 */

#ifndef PID_H_
#define PID_H_


float pid_calculate_roll(float IMU_roll_value, int timer_value);
float pid_calculate_pitch(float IMU_pitch_value, int timer_value);



void p_up();
void p_down();
void i_up();
void i_down();
void d_up();
void d_down();

#endif /* PID_H_ */
