/*
 * bno055_top.h
 *
 *  Created on: 19 Sep 2019
 *      Author: nick_savva
 */

#ifndef BNO055_TOP_H_
#define BNO055_TOP_H_

#define	I2C_BUFFER_LEN 8
#define I2C0 5

#include "bno055.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

static I2C_HandleTypeDef BNO055_i2c_handle;

struct bno055_t bno055;

#define	BNO055_I2C_BUS_WRITE_ARRAY_INDEX	((u8)1)
/*	\Brief: The API is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *   will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *   which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*	\Brief: The API is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *   will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *	will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*
 * \Brief: I2C init routine
 */
s8 I2C_routine(void);
void BNO055_delay_msek(u32 msek);

uint8_t operation_mode_read = 0xFF;

/* Variable used to return value of
	communication routine*/
s32 comres = BNO055_ERROR;
/* variable used to set the power mode of the sensor*/
u8 power_mode = BNO055_INIT_VALUE;

/*********read raw accel data***********/
/* variable used to read the accel x data */
s16 accel_datax = BNO055_INIT_VALUE;
/* variable used to read the accel y data */
s16 accel_datay = BNO055_INIT_VALUE;
/* variable used to read the accel z data */
s16 accel_dataz = BNO055_INIT_VALUE;
/* variable used to read the accel xyz data */
struct bno055_accel_t accel_xyz;

/*********read raw mag data***********/
/* variable used to read the mag x data */
s16 mag_datax  = BNO055_INIT_VALUE;
/* variable used to read the mag y data */
s16 mag_datay  = BNO055_INIT_VALUE;
/* variable used to read the mag z data */
s16 mag_dataz  = BNO055_INIT_VALUE;
/* structure used to read the mag xyz data */
struct bno055_mag_t mag_xyz;

/***********read raw gyro data***********/
/* variable used to read the gyro x data */
s16 gyro_datax = BNO055_INIT_VALUE;
/* variable used to read the gyro y data */
s16 gyro_datay = BNO055_INIT_VALUE;
/* variable used to read the gyro z data */
s16 gyro_dataz = BNO055_INIT_VALUE;
/* structure used to read the gyro xyz data */
struct bno055_gyro_t gyro_xyz;
/*************read raw Euler data************/
/* variable used to read the euler h data */
s16 euler_data_h = BNO055_INIT_VALUE;
/* variable used to read the euler r data */
s16 euler_data_r = BNO055_INIT_VALUE;
/* variable used to read the euler p data */
s16 euler_data_p = BNO055_INIT_VALUE;

/* variable used to read the euler h data output
	as degree or radians*/
double d_euler_data_h = BNO055_INIT_VALUE;
/* variable used to read the euler r data output
	as degree or radians*/
double d_euler_data_r = BNO055_INIT_VALUE;
/* variable used to read the euler p data output
	as degree or radians*/
double d_euler_data_p = BNO055_INIT_VALUE;

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values


void bno055_top_init(I2C_HandleTypeDef handle){


	//Copy i2c handle variable
	memcpy(&BNO055_i2c_handle, &handle, sizeof(handle));


	//Check if device is connected properly and responding first
	if (HAL_I2C_IsDeviceReady(&BNO055_i2c_handle, BNO055_I2C_ADDR1, 2, 100) == HAL_OK) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); //Toggle LED on if so
	}
	else{
		while(1);
	}

	I2C_routine();

	comres = bno055_init(&bno055);

 	/*	For initializing the BNO sensor it is required to the operation mode
		of the sensor as NORMAL
		Normal mode can set from the register
		Page - page0
		register - 0x3E
		bit positions - 0 and 1*/
	power_mode = BNO055_POWER_MODE_NORMAL;
	/* set the power mode as NORMAL*/
	comres += bno055_set_power_mode(power_mode);

	//Set operation to config mode as editing a few registers
	comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);

	//Re map X & Y axes so roll and pitch axes are correct
	comres += bno055_set_axis_remap_value(BNO055_REMAP_X_Y);

	comres += bno055_set_accel_range(BNO055_ACCEL_RANGE_2G);
	comres += bno055_set_gyro_range(BNO055_GYRO_RANGE_250DPS);

	//After config set operation mode to NDOF to begin getting orientation
	comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG);
}


void bno055_getRawAMG(){


	/*	Raw accel X, Y and Z data can read from the register
		page - page 0
		register - 0x08 to 0x0D*/
	comres += bno055_read_accel_x(&accel_datax);
	comres += bno055_read_accel_y(&accel_datay);
	comres += bno055_read_accel_z(&accel_dataz);

	/*	Raw mag X, Y and Z data can read from the register
		page - page 0
		register - 0x0E to 0x13*/
	comres += bno055_read_mag_x(&mag_datax);
	comres += bno055_read_mag_y(&mag_datay);
	comres += bno055_read_mag_z(&mag_dataz);

	/*	Raw gyro X, Y and Z data can read from the register
		page - page 0
		register - 0x14 to 0x19*/
	comres += bno055_read_gyro_x(&gyro_datax);
	comres += bno055_read_gyro_y(&gyro_datay);
	comres += bno055_read_gyro_z(&gyro_dataz);


}


void bno055_getEurler(float *roll, float *pitch, float *yaw){


	comres += bno055_convert_double_euler_h_deg(&d_euler_data_h);
	comres += bno055_convert_double_euler_r_deg(&d_euler_data_r);
	comres += bno055_convert_double_euler_p_deg(&d_euler_data_p);


	*roll = d_euler_data_r;
	*pitch = d_euler_data_p;
	*yaw = d_euler_data_h;
}







/*	\Brief: The API is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *   will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{

	uint8_t data_write[I2C_BUFFER_LEN];
	data_write[0] = reg_addr;

	for(int i = 1; i <= cnt; i++){
		data_write[i] = *reg_data;
		reg_data++;
	}


	if(HAL_I2C_Master_Transmit(&BNO055_i2c_handle, dev_addr, data_write, cnt+1, 2) != HAL_OK){
		while(1);
	}

	return 0;

}


s8 I2C_routine(void)
{
	bno055.bus_write = BNO055_I2C_bus_write;
	bno055.bus_read = BNO055_I2C_bus_read;
	bno055.delay_msec = BNO055_delay_msek;
	bno055.dev_addr = BNO055_I2C_ADDR1;

	return BNO055_INIT_VALUE;
}

/*	\Brief: The API is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *  will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *   which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{

	uint8_t data_write[I2C_BUFFER_LEN];
	data_write[0] = reg_addr;

	//Send adress of register ONLY
	if(HAL_I2C_Master_Transmit(&BNO055_i2c_handle, dev_addr, data_write, 1, 2)!= HAL_OK){
		while(1);
	}
	if(HAL_I2C_Master_Receive(&BNO055_i2c_handle, dev_addr, reg_data, cnt, 2) != HAL_OK){
		while(1);
	}

	return 0;

}
/*	Brief : The delay routine
 *	\param : delay in ms
 */
void BNO055_delay_msek(u32 msek)
{
	HAL_Delay(msek);
}




#endif /* BNO055_TOP_H_ */
