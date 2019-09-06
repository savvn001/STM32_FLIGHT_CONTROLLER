/*
 * IMU_eMP.h
 *
 *  Created on: Sep 6, 2019
 *      Author: nick_savva
 */

#ifndef IMU_EMD_H_
#define IMU_EMD_H_

#define COMPASS_ENABLED


#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
//#include "packet.h"


int init(){


	inv_error_t result;
	unsigned char accel_fsr,  new_temp = 0;
	unsigned short gyro_rate, gyro_fsr;
	unsigned long timestamp;
	struct int_param_s int_param;

#ifdef COMPASS_ENABLED
	unsigned char new_compass = 0;
	unsigned short compass_fsr;
#endif

	result = mpu_init(&int_param);
	if (result) {
		MPL_LOGE("Could not initialize gyro.\n");
	}



}



#endif /* IMU_EMD_H_ */
