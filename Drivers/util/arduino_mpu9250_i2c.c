/******************************************************************************
arduino_mpu9250_i2c.cpp - MPU-9250 Digital Motion Processor Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

This library implements motion processing functions of Invensense's MPU-9250.
It is based on their Emedded MotionDriver 6.12 library.
	https://www.invensense.com/developers/software-downloads/

Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
******************************************************************************/
#include "arduino_mpu9250_i2c.h"
#include "stm32f4xx_hal.h"

I2C_HandleTypeDef hi2c2;


int arduino_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
	//Shift left 1 bit as STM HAL takes 8 bit slave address
		unsigned char slave_addr_shift = slave_addr << 1;


		unsigned char data_write[32];

		data_write[0] = reg_addr;

		for (int i = 1; i <= length; ++i) {
			data_write[i] = *data;
			data++;
		}

		if(HAL_I2C_Master_Transmit(&hi2c2, 0xD0, data_write, length+1, 10) == HAL_OK ){
			return 0;
		}
		else{
			return 1;
		}
}

int arduino_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
	unsigned char slave_addr_shift = slave_addr << 1;

		unsigned char data_read[32];

		unsigned char data_write[1];
		data_write[0] = reg_addr;

		//Send adress of register ONLY
		HAL_I2C_Master_Transmit(&hi2c2, 0xD0, data_write, 1, 10);
		//Then read from register
		HAL_I2C_Master_Receive(&hi2c2, 0xD1, data, length, 10);


		return 0;
}
