/*
 * MPL_HAL_STM32F4.h
 *
 *  Created on: Sep 6, 2019
 *      Author: nick_savva
 */

#ifndef EMD_MPL_HAL_STM32F4_H_
#define EMD_MPL_HAL_STM32F4_H_

#include "stm32f4xx_hal.h"
I2C_HandleTypeDef hi2c2;


#define i2c_write   stm32f4_i2c_write
#define i2c_read    stm32f4_i2c_read
#define delay_ms    stm32_delay_ms
#define get_ms 		stm32__get_clock_ms
#define log_i		 MPL_LOGI
#define log_e 		MPL_LOGE


int stm32f4_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data){


	unsigned char data_write[32];

	data_write[0] = reg_addr;


	if(HAL_I2C_Master_Transmit(&hi2c2, slave_addr, data_write, length, 10) ){
		return 0;
	}
	else{
		return 1;
	}


}

int stm32f4_i2c_read(){
	return 0;
}



int stm32_delay_ms(){

	return 0;
}

int stm32__get_clock_ms(){
	return 0;
}

int MPL_LOGI(){
	return 0;
}

int MPL_LOGE(){
	return 0;
}
#endif /* EMD_MPL_HAL_STM32F4_H_ */
