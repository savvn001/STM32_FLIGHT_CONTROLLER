#include "controlLoop.h"

#include "../Drivers/PID.h"
#include <stdbool.h>
#include <stdio.h>
#include "arm_math.h"

#include "../Drivers/IMU.h"

#include "tim.h"


float imu_roll = 0;
float imu_pitch = 0;
float imu_yaw = 0;


//////////////////////////////////// PID variables /////////////////////////
float roll_setpoint = 0;
float pitch_setpoint = 0;
float yaw_setpoint = 0;
float pid_output_roll = 0;
float pid_output_pitch = 0;
float pid_output_yaw = 0;

int tim11_count = 0;

int N = 0;

uint16_t throttle = 0;

int esc1_total = 0;
int esc2_total = 0;
int esc3_total = 0;
int esc4_total = 0;

bool getRPY_flag = 0;


float ax,ay,az;
float gx,gy,gz;
float mx,my,mz;
float temp;

void CL_init() {


#if NRF24

	RF_init();

#endif

#if IMU

		//Start timer 11, used for integral calculations
		HAL_TIM_Base_Start(&htim11);


		if (imu_init(&hi2c2) == IMU_SUCCESS) {
			imu_calibrate();
		}

		for (int i = 0; i < 31; ++i) {
			AckPayload_0[i] = 0;
			AckPayload_1[i] = 0;

		}
	#endif

	/////////////////////////////////////////////////////////////////
	////////////////////////// Init timers //////////////////////////
	/////////////////////////////////////////////////////////////////

	//Start up PWMs
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	//ARM_ESCs(); //Force arming sequence

	PWM1_Set(2500);
	PWM2_Set(2500);
	PWM3_Set(2500);
	PWM4_Set(2500);

}


/* ------------------------------ MAIN CONTROL LOOP --------------------------------------
 *
 * Calculate the required pulse widths to set individual motor speeds based off of IMU
 * readings and instructions from controller.
 *
 * For the QAV210 kit the orientation is as so:
 *
 * (2 CCW)    (4 CW)
 *		\	  /
 *		   |
 *		   |
 *		   |
 *		/     \
 * (1 CW)     (3 CCW)
 *
 * This function gets called by the GPIO_EXTI callback when the PWM_RE_INT_Pin triggers an interrupt,
 * which is on the rising edge of every PWM pulse.
 */
void CL_main() {


#if NRF24
	RF_TxRx(&throttle, &pitch_setpoint, &roll_setpoint, &yaw_setpoint, imu_roll, imu_pitch, imu_yaw);
#endif


#if IMU
		//Calculate roll, pitch & yaw using IMU readings
		tim11_count = htim11.Instance->CNT; //read TIM11 counter value, used for integral calculations

		int tim1 = tim11_count;

		calc_RollPitchYaw(tim11_count);


		tim11_count = htim11.Instance->CNT; //read TIM11 counter value, used for integral calculations
		int tim2 = tim11_count;

		volatile int deltat = tim2-tim1;


		imu_pitch = get_pitch();
		imu_roll = get_roll();
		imu_yaw = get_yaw();
#endif
		if (Rx_Data.airmode) {
			/*******    Pitch PID calculation  ********/
			pid_output_pitch = pid_calculate_pitch(imu_pitch, 0, 0);

			/*******    Roll PID calculation  ********/

			pid_output_roll = pid_calculate_roll(imu_roll, 0, roll_setpoint);

			/*******    Yaw PID calculation  ********/

			pid_output_yaw = pid_calculate_yaw(imu_yaw, 0, yaw_setpoint);
		} else {
			pid_output_roll = 0;
			pid_output_pitch = 0;
			pid_output_yaw = 0;
			reset_pid_roll();
			reset_pid_pitch();
			reset_pid_yaw();
		}

		/*** For tuning PID, store in buffer to print to PC later ***/
#if PID_TUNE_DEBUG
		if (print_buffer_index < SAMPLES) {
			PID_print_buffer[print_buffer_index] = roll;
			IMU_print_buffer[print_buffer_index] = roll;

			print_buffer_index++;
		} else {
			printToPC();
		}
#endif

		//Calculate new pulse width values
		esc1_total = throttle - (int) pid_output_roll - (int) pid_output_pitch;
		esc2_total = throttle - (int) pid_output_roll + (int) pid_output_pitch;
		esc3_total = (throttle) + (int) pid_output_roll
				- (int) pid_output_pitch;
		esc4_total = (throttle) + (int) pid_output_roll
				+ (int) pid_output_pitch;

		//Clip PWM values to make sure they don't go outside of range
		if (esc1_total < ESC_MIN) {
			esc1_total = ESC_MIN;
		}
		if (esc1_total > ESC_MAX) {
			esc1_total = ESC_MAX;
		}
		if (esc2_total < ESC_MIN) {
			esc2_total = ESC_MIN;
		}
		if (esc2_total > ESC_MAX) {
			esc2_total = ESC_MAX;
		}
		if (esc3_total < ESC_MIN) {
			esc3_total = ESC_MIN;
		}
		if (esc3_total > ESC_MAX) {
			esc3_total = ESC_MAX;
		}
		if (esc4_total < ESC_MIN) {
			esc4_total = ESC_MIN;
		}
		if (esc4_total > ESC_MAX) {
			esc4_total = ESC_MAX;
		}
#if MOTORS
		//Load new pulse widths into ESCs
		PWM1_Set(esc1_total); //PWM1 = Back left, CW
		PWM2_Set(esc2_total); //PWM2 = Front left, CCW
		PWM3_Set(esc3_total); //PWM3 = Back right, CCW
		PWM4_Set(esc4_total); //PWM4 = Front right, CW

#endif



}



/*
 *  Kill function disables PWM outputs, turning off motors
 */
void kill() {
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
}

/*
 *  In case connection to transmitter is lost, slowly power down motors
 *  in order to gently lower quadcopter as opposed to suddenly shutting them off
 *
 */
void lostConnection() {

	Rx_Data.airmode = 1;

	//Reset joystick positions to centre
	roll_setpoint = 0;
	pitch_setpoint = 0;

	//Decrease throttle down to min value
	if (throttle > ESC_MIN) {
		throttle--;
		NRF24_DelayMicroSeconds(5000);
	} else {
		//Then turn off motors fully to be sure
		kill();
	}

}

/*
 * Reset NRF24 module
 */
void resetNRF24() {

//	NRF24_powerDown();
//	DWT_Init(); //Enable some of the MCUs special registers so we can get microsecond (us) delays
//	NRF24_begin(GPIOB, nrf_CSN_PIN, nrf_CE_PIN, hspi2);
//	//nrf24_DebugUART_Init(huart2);
//	NRF24_setAutoAck(true);
//	NRF24_enableAckPayload();
//	NRF24_openReadingPipe(1, TxpipeAddrs);
//	NRF24_startListening();

}


//These 4 functions set the PWM duty cycles
void PWM1_Set(uint16_t value) {
	htim4.Instance->CCR1 = value;
}

void PWM2_Set(uint16_t value) {
	htim4.Instance->CCR2 = value;

}

void PWM3_Set(uint16_t value) {
	htim4.Instance->CCR3 = value;
}

void PWM4_Set(uint16_t value) {
	htim4.Instance->CCR4 = value;
}




