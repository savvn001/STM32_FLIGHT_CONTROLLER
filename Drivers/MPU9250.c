/* 06/16/2017 Copyright Tlera Corporation
 *
 *  Created by Kris Winer
 *
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor,
 getting properly scaled accelerometer, gyroscope, and magnetometer data out.
 Addition of 9 DoF sensor fusion using open source Madgwick filter algorithm.
 Sketch runs on the 3.3 V Dragonfly STM32L476 Breakout Board.

 Library may be used freely and without limit with attribution.

 */

#include "MPU9250.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"

I2C_HandleTypeDef hi2c2;

// MPU9250 Configuration
// Specify sensor full scale
/* Choices are:
 *  Gscale: GFS_250 == 250 dps, GFS_500 DPS == 500 dps, GFS_1000 == 1000 dps, and GFS_2000DPS == 2000 degrees per second gyro full scale
 *  Ascale: AFS_2G == 2 g, AFS_4G == 4 g, AFS_8G == 8 g, and AFS_16G == 16 g accelerometer full scale
 *  Mscale: MFS_14BITS == 0.6 mG per LSB and MFS_16BITS == 0.15 mG per LSB
 *  Mmode: Mmode == M_8Hz for 8 Hz data rate or Mmode = M_100Hz for 100 Hz data rate
 *  (1 + sampleRate) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so
 *  sampleRate = 0x00 means 1 kHz sample rate for both accel and gyro, 0x04 means 200 Hz, etc.
 */
uint8_t Gscale = GFS_250DPS, Ascale = AFS_2G, Mscale = MFS_16BITS,
Mmode = M_100Hz, sampleRate = 0x00;

float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
float motion = 0; // check on linear acceleration to determine motion
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError = 0.6981317007977318; // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = 0; // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = 0.7236012545582676434;   // compute beta
float zeta = 0; // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
bool wakeup;

// Pin definitions
int intPin1 = 9;  //  MPU9250 1 interrupt
int intPin2 = 8;  //  MPU9250 2 interrupt
int myLed = 13; // red led

bool intFlag1 = false;
bool intFlag2 = false;
bool newMagData = false;

int16_t MPU9250Data1[7], MPU9250Data2[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
int16_t magCount1[3], magCount2[3]; // Stores the 16-bit signed magnetometer sensor output
float magCalibration1[3] = { 0, 0, 0 }, magCalibration2[3] = { 0, 0, 0 }; // Factory mag calibration and mag bias
float temperature1, temperature2; // Stores the MPU9250 internal chip temperature in degrees Celsius
float selfTest[6];    // holds results of gyro and accelerometer self test

// These can be measured once and entered here or can be calculated each time the device is powered on
float gyroBias1[3] = { 0.96, -0.21, 0.12 }, accelBias1[3] = { 0.00299, -0.00916,
		0.00952 };
float gyroBias2[3] = { 0.96, -0.21, 0.12 }, accelBias2[3] = { 0.00299, -0.00916,
		0.00952 };
float magBias1[3] = { 71.04, 122.43, -36.90 }, magScale1[3] =
		{ 1.01, 1.03, 0.96 }; // Bias corrections for gyro and accelerometer
float magBias2[3] = { 71.04, 122.43, -36.90 }, magScale2[3] =
		{ 1.01, 1.03, 0.96 }; // Bias corrections for gyro and accelerometer

uint32_t delt_t1, delt_t2 = 0;            // used to control display output rate
uint32_t count1 = 0, sumCount1 = 0, count2 = 0, sumCount2 = 0; // used to control display output rate
float pitch1, yaw1, roll1, pitch2, yaw2, roll2;          // absolute orientation
float a12, a22, a31, a32, a33; // rotation matrix coefficients for Euler angles and gravity components
float A12, A22, A31, A32, A33; // rotation matrix coefficients for Euler angles and gravity components
float deltat1 = 0.0f, sum1 = 0.0f, deltat2 = 0.0f, sum2 = 0.0f; // integration interval for both filter schemes
uint32_t lastUpdate1 = 0, lastUpdate2 = 0; // used to calculate integration interval
uint32_t Now1 = 0, Now2 = 0;           // used to calculate integration interval

float ax1, ay1, az1, gx1, gy1, gz1, mx1, my1, mz1; // variables to hold latest sensor data values
float ax2, ay2, az2, gx2, gy2, gz2, mx2, my2, mz2; // variables to hold latest sensor data values
float lin_ax1, lin_ay1, lin_az1; // linear acceleration (acceleration with gravity component subtracted)
float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };    // vector to hold quaternion
float lin_ax2, lin_ay2, lin_az2; // linear acceleration (acceleration with gravity component subtracted)
float Q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };    // vector to hold quaternion

uint8_t getMPU9250ID(uint8_t MPUnum) {
	uint8_t c = readByte(MPUnum, WHO_AM_I_MPU9250); // Read WHO_AM_I register for MPU-9250
	return c;
}

uint8_t getAK8963CID(uint8_t MPUnum) {
//  uint8_t c = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for MPU-9250
	writeByte(MPUnum, USER_CTRL, 0x20);    // Enable I2C Master mode
	writeByte(MPUnum, I2C_MST_CTRL, 0x0D); // I2C configuration multi-master I2C 400KHz

	writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80); // Set the I2C slave address of AK8963 and set for read.
	writeByte(MPUnum, I2C_SLV0_REG, WHO_AM_I_AK8963); // I2C slave 0 register address from where to begin data transfer
	writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);    // Enable I2C and transfer 1 byte
	HAL_Delay(10);
	uint8_t c = readByte(MPUnum, EXT_SENS_DATA_00);    // Read the WHO_AM_I byte
	return c;
}

float getMres(uint8_t Mscale) {
	switch (Mscale) {
	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
	case MFS_14BITS:
		_mRes = 10.0f * 4912.0f / 8190.0f; // Proper scale to return milliGauss
		return _mRes;
		break;
	case MFS_16BITS:
		_mRes = 10.0f * 4912.0f / 32760.0f; // Proper scale to return milliGauss
		return _mRes;
		break;
	}
}

float getGres(uint8_t Gscale) {
	switch (Gscale) {
	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
	case GFS_250DPS:
		_gRes = 250.0 / 32768.0;
		return _gRes;
		break;
	case GFS_500DPS:
		_gRes = 500.0 / 32768.0;
		return _gRes;
		break;
	case GFS_1000DPS:
		_gRes = 1000.0 / 32768.0;
		return _gRes;
		break;
	case GFS_2000DPS:
		_gRes = 2000.0 / 32768.0;
		return _gRes;
		break;
	}
}

float getAres(uint8_t Ascale) {
	switch (Ascale) {
	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
	// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case AFS_2G:
		_aRes = 2.0f / 32768.0f;
		return _aRes;
		break;
	case AFS_4G:
		_aRes = 4.0f / 32768.0f;
		return _aRes;
		break;
	case AFS_8G:
		_aRes = 8.0f / 32768.0f;
		return _aRes;
		break;
	case AFS_16G:
		_aRes = 16.0f / 32768.0f;
		return _aRes;
		break;
	}
}

void accelWakeOnMotion(uint8_t MPUnum) {
	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	uint8_t c = readByte(MPUnum, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	c = c | 0x01;  // Set accelerometer rate to 1 kHz and bandwidth to 184 Hz
	writeByte(MPUnum, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	writeByte(MPUnum, INT_PIN_CFG, 0x12); // INT is 50 microsecond pulse and any read to clear
	writeByte(MPUnum, INT_ENABLE, 0x41); // Enable data ready (bit 0) and wake on motion (bit 6)  interrupt

	// enable wake on motion detection logic (bit 7) and compare current sample to previous sample (bit 6)
	writeByte(MPUnum, MOT_DETECT_CTRL, 0xC0);

	// set accel threshold for wake up at  mG per LSB, 1 - 255 LSBs == 0 - 1020 mg), pic 0x19 for 25 mg
	writeByte(MPUnum, WOM_THR, 0x19);

	// set sample rate in low power mode
	/* choices are 0 == 0.24 Hz, 1 == 0.49 Hz, 2 == 0.98 Hz, 3 == 1.958 Hz, 4 == 3.91 Hz, 5 == 7.81 Hz
	 *             6 == 15.63 Hz, 7 == 31.25 Hz, 8 == 62.50 Hz, 9 = 125 Hz, 10 == 250 Hz, and 11 == 500 Hz
	 */
	writeByte(MPUnum, LP_ACCEL_ODR, 0x02);

	c = readByte(MPUnum, PWR_MGMT_1);
	writeByte(MPUnum, PWR_MGMT_1, c | 0x20); // Write bit 5 to enable accel cycling

	gyromagSleep(MPUnum);
	HAL_Delay(100); // Wait for all registers to reset

}

void gyromagSleep(uint8_t MPUnum) {
	uint8_t temp = 0;
	writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80); // Set the I2C slave address of AK8963 and set for read.
	writeByte(MPUnum, I2C_SLV0_REG, AK8963_CNTL); // I2C slave 0 register address from where to begin data transfer
	writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);    // Enable I2C and transfer 1 byte
	HAL_Delay(10);
	temp = readByte(MPUnum, EXT_SENS_DATA_00);

	writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS); // Set the I2C slave address of AK8963 and set for write.
	writeByte(MPUnum, I2C_SLV0_REG, AK8963_CNTL); // I2C slave 0 register address from where to begin data transfer
	writeByte(MPUnum, I2C_SLV0_DO, temp & ~(0x0F));         // Power down AK8963
	writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);    // Enable I2C and transfer 1 byte
	HAL_Delay(10);

	temp = readByte(MPUnum, PWR_MGMT_1);
	writeByte(MPUnum, PWR_MGMT_1, temp | 0x10); // Write bit 4 to enable gyro standby
	HAL_Delay(10); // Wait for all registers to reset
}

void gyromagWake(uint8_t MPUnum, uint8_t Mmode) {
	uint8_t temp = 0;
	writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80); // Set the I2C slave address of AK8963 and set for read.
	writeByte(MPUnum, I2C_SLV0_REG, AK8963_CNTL); // I2C slave 0 register address from where to begin data transfer
	writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);    // Enable I2C and transfer 1 byte
	HAL_Delay(10);
	temp = readByte(MPUnum, EXT_SENS_DATA_00);

	writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS); // Set the I2C slave address of AK8963 and set for write.
	writeByte(MPUnum, I2C_SLV0_REG, AK8963_CNTL); // I2C slave 0 register address from where to begin data transfer
	writeByte(MPUnum, I2C_SLV0_DO, temp | Mmode); // Reset normal mode for  magnetometer
	writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);    // Enable I2C and transfer 1 byte
	HAL_Delay(10);
	temp = readByte(MPUnum, PWR_MGMT_1);
	writeByte(MPUnum, PWR_MGMT_1, 0x01);    // return gyro and accel normal mode
	HAL_Delay(10); // Wait for all registers to reset
}

void resetMPU9250(uint8_t MPUnum) {
	// reset device
	writeByte(MPUnum, PWR_MGMT_1, 0x80); // Set bit 7 to reset MPU9250
	HAL_Delay(100); // Wait for all registers to reset
}

void readMPU9250Data(uint8_t MPUnum, int16_t * destination) {
	uint8_t rawData[14];  // x/y/z accel register data stored here
	readBytes(MPUnum, ACCEL_XOUT_H, 14, &rawData[0]); // Read the 14 raw data registers into data array
	destination[0] = ((int16_t) rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t) rawData[2] << 8) | rawData[3];
	destination[2] = ((int16_t) rawData[4] << 8) | rawData[5];
	destination[3] = ((int16_t) rawData[6] << 8) | rawData[7];
	destination[4] = ((int16_t) rawData[8] << 8) | rawData[9];
	destination[5] = ((int16_t) rawData[10] << 8) | rawData[11];
	destination[6] = ((int16_t) rawData[12] << 8) | rawData[13];
}

void readAccelData(uint8_t MPUnum, int16_t * destination) {
	uint8_t rawData[6];  // x/y/z accel register data stored here
	readBytes(MPUnum, ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
	destination[0] = ((int16_t) rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t) rawData[2] << 8) | rawData[3];
	destination[2] = ((int16_t) rawData[4] << 8) | rawData[5];
}

void readGyroData(uint8_t MPUnum, int16_t * destination) {
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(MPUnum, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t) rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t) rawData[2] << 8) | rawData[3];
	destination[2] = ((int16_t) rawData[4] << 8) | rawData[5];
}

bool checkNewMagData(uint8_t MPUnum) {
	bool test;
	test = (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01);
	writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80); // Set the I2C slave address of AK8963 and set for read.
	writeByte(MPUnum, I2C_SLV0_REG, AK8963_ST1); // I2C slave 0 register address from where to begin data transfer
	writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);    // Enable I2C and transfer 1 byte
	HAL_Delay(2);
	test = (readByte(MPUnum, EXT_SENS_DATA_00) & 0x01); // Check data ready status byte
	return test;
}

bool checkNewAccelGyroData(uint8_t MPUnum) {
	bool test;
	test = (readByte(MPUnum, INT_STATUS) & 0x01);
	return test;
}

bool checkWakeOnMotion(uint8_t MPUnum) {
	bool test;
	test = (readByte(MPUnum, INT_STATUS) & 0x40);
	return test;
}

void readMagData(uint8_t MPUnum, int16_t * destination) {
	uint8_t rawData[7]; // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
//  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
	writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80); // Set the I2C slave address of AK8963 and set for read.
	writeByte(MPUnum, I2C_SLV0_REG, AK8963_XOUT_L); // I2C slave 0 register address from where to begin data transfer
	writeByte(MPUnum, I2C_SLV0_CTRL, 0x87);       // Enable I2C and read 7 bytes
//   delay(10);
	readBytes(MPUnum, EXT_SENS_DATA_00, 7, &rawData[0]); // Read the x-, y-, and z-axis calibration values
	uint8_t c = rawData[6]; // End data read by reading ST2 register
	if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
		destination[0] = ((int16_t) rawData[1] << 8) | rawData[0]; // Turn the MSB and LSB into a signed 16-bit value
		destination[1] = ((int16_t) rawData[3] << 8) | rawData[2]; // Data stored as little Endian
		destination[2] = ((int16_t) rawData[5] << 8) | rawData[4];
	}
}

int16_t readGyroTempData(uint8_t MPUnum) {
	uint8_t rawData[2];  // x/y/z gyro register data stored here
	readBytes(MPUnum, TEMP_OUT_H, 2, &rawData[0]); // Read the two raw data registers sequentially into data array
	return ((int16_t) rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a 16-bit value
}

void initAK8963Slave(uint8_t MPUnum, uint8_t Mscale, uint8_t Mmode,
		float * magCalibration) {
	// First extract the factory calibration for each magnetometer axis
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	_Mmode = Mmode;

	writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS); // Set the I2C slave address of AK8963 and set for write.
	writeByte(MPUnum, I2C_SLV0_REG, AK8963_CNTL2); // I2C slave 0 register address from where to begin data transfer
	writeByte(MPUnum, I2C_SLV0_DO, 0x01);                       // Reset AK8963
	writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);       // Enable I2C and write 1 byte
	HAL_Delay(50);
	writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS); // Set the I2C slave address of AK8963 and set for write.
	writeByte(MPUnum, I2C_SLV0_REG, AK8963_CNTL); // I2C slave 0 register address from where to begin data transfer
	writeByte(MPUnum, I2C_SLV0_DO, 0x00);             // Power down magnetometer
	writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);       // Enable I2C and write 1 byte
	HAL_Delay(50);
	writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS); // Set the I2C slave address of AK8963 and set for write.
	writeByte(MPUnum, I2C_SLV0_REG, AK8963_CNTL); // I2C slave 0 register address from where to begin data transfer
	writeByte(MPUnum, I2C_SLV0_DO, 0x0F);                     // Enter fuze mode
	writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);       // Enable I2C and write 1 byte
	HAL_Delay(50);

	writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80); // Set the I2C slave address of AK8963 and set for read.
	writeByte(MPUnum, I2C_SLV0_REG, AK8963_ASAX); // I2C slave 0 register address from where to begin data transfer
	writeByte(MPUnum, I2C_SLV0_CTRL, 0x83);       // Enable I2C and read 3 bytes
	HAL_Delay(50);
	readBytes(MPUnum, EXT_SENS_DATA_00, 3, &rawData[0]); // Read the x-, y-, and z-axis calibration values
	magCalibration[0] = (float) (rawData[0] - 128) / 256.0f + 1.0f; // Return x-axis sensitivity adjustment values, etc.
	magCalibration[1] = (float) (rawData[1] - 128) / 256.0f + 1.0f;
	magCalibration[2] = (float) (rawData[2] - 128) / 256.0f + 1.0f;

	writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS); // Set the I2C slave address of AK8963 and set for write.
	writeByte(MPUnum, I2C_SLV0_REG, AK8963_CNTL); // I2C slave 0 register address from where to begin data transfer
	writeByte(MPUnum, I2C_SLV0_DO, 0x00);             // Power down magnetometer
	writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);    // Enable I2C and transfer 1 byte
	HAL_Delay(50);

	writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS); // Set the I2C slave address of AK8963 and set for write.
	writeByte(MPUnum, I2C_SLV0_REG, AK8963_CNTL); // I2C slave 0 register address from where to begin data transfer
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	writeByte(MPUnum, I2C_SLV0_DO, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
	writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);    // Enable I2C and transfer 1 byte
	HAL_Delay(50);
}

void initMPU9250(uint8_t MPUnum, uint8_t Ascale, uint8_t Gscale,
		uint8_t sampleRate) {
	// wake up device
	writeByte(MPUnum, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
	HAL_Delay(100); // Wait for all registers to reset

	// get stable time source
	writeByte(MPUnum, PWR_MGMT_1, 0x01); // Auto select clock source to be PLL gyroscope reference if ready else
	HAL_Delay(200);

	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	writeByte(MPUnum, CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	writeByte(MPUnum, SMPLRT_DIV, sampleRate); // Use a 200 Hz rate; a rate consistent with the filter update rate
											   // determined inset in CONFIG above

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = readByte(MPUnum, GYRO_CONFIG); // get current GYRO_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x02; // Clear Fchoice bits [1:0]
	c = c & ~0x18; // Clear AFS bits [4:3]
	c = c | Gscale << 3; // Set full scale range for the gyro
	// c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	writeByte(MPUnum, GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register

	// Set accelerometer full-scale range configuration
	c = readByte(MPUnum, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x18;  // Clear AFS bits [4:3]
	c = c | Ascale << 3; // Set full scale range for the accelerometer
	writeByte(MPUnum, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = readByte(MPUnum, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	writeByte(MPUnum, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	writeByte(MPUnum, INT_PIN_CFG, 0x10); // INT is 50 microsecond pulse and any read to clear
	writeByte(MPUnum, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	HAL_Delay(100);

	writeByte(MPUnum, USER_CTRL, 0x20);          // Enable I2C Master mode
	writeByte(MPUnum, I2C_MST_CTRL, 0x1D); // I2C configuration STOP after each transaction, master I2C bus at 400 KHz
	writeByte(MPUnum, I2C_MST_DELAY_CTRL, 0x81); // Use blocking data retreival and enable delay for mag sample rate mismatch
	writeByte(MPUnum, I2C_SLV4_CTRL, 0x01); // Delay mag data retrieval to once every other accel/gyro data sample
}

void magcalMPU9250(uint8_t MPUnum, float * dest1, float * dest2) {
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = { 0, 0, 0 }, mag_scale[3] = { 0, 0, 0 };
	int16_t mag_max[3] = { -32767, -32767, -32767 }, mag_min[3] = { 32767,
			32767, 32767 }, mag_temp[3] = { 0, 0, 0 };
	uint8_t rawData[7] = { 0, 0, 0, 0, 0, 0, 0 }, magCalibration[3] =
			{ 0, 0, 0 };

	printf("Mag Calibration for: 0x  \r\n ");
	// printf(MPUnum, HEX);
	printf("Wave device in a figure eight until done!  \r\n ");
	HAL_Delay(4000);

// shoot for ~fifteen seconds of mag data
	if (_Mmode == 0x02)
		sample_count = 128; // at 8 Hz ODR, new mag data is available every 125 ms
	if (_Mmode == 0x06)
		sample_count = 500;	//1500; // at 100 Hz ODR, new mag data is available every 10 ms



	for (ii = 0; ii < sample_count; ii++) {
		writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80); // Set the I2C slave address of AK8963 and set for read.
		writeByte(MPUnum, I2C_SLV0_REG, AK8963_XOUT_L); // I2C slave 0 register address from where to begin data transfer
		writeByte(MPUnum, I2C_SLV0_CTRL, 0x87);   // Enable I2C and read 7 bytes
		if (_Mmode == 0x02)
			HAL_Delay(125); // at 8 Hz ODR, new mag data is available every 125 ms
		if (_Mmode == 0x06)
			HAL_Delay(10); // at 100 Hz ODR, new mag data is available every 10 ms
		readBytes(MPUnum, EXT_SENS_DATA_00, 7, &rawData[0]); // Read the x-, y-, and z-axis calibration values
		mag_temp[0] = ((int16_t) rawData[1] << 8) | rawData[0]; // Turn the MSB and LSB into a signed 16-bit value
		mag_temp[1] = ((int16_t) rawData[3] << 8) | rawData[2]; // Data stored as little Endian
		mag_temp[2] = ((int16_t) rawData[5] << 8) | rawData[4];

		for (int jj = 0; jj < 3; jj++) {
			if (mag_temp[jj] > mag_max[jj])
				mag_max[jj] = mag_temp[jj];
			if (mag_temp[jj] < mag_min[jj])
				mag_min[jj] = mag_temp[jj];
		}
	}


	printf("mag x min/max:  \r\n ");
	printf("%d \n\r", mag_max[0]);
	printf("%d \n\r", mag_min[0]);
	printf("mag y min/max:  \r\n ");
	printf("%d \n\r", mag_max[1]);
	printf("%d \n\r", mag_min[1]);
	printf("mag z min/max:  \r\n ");
	printf("%d \n\r", mag_max[2]);
	printf("%d \n\r", mag_min[2]);

	// Get hard iron correction
	mag_bias[0] = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
	mag_bias[1] = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
	mag_bias[2] = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

	writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80); // Set the I2C slave address of AK8963 and set for read.
	writeByte(MPUnum, I2C_SLV0_REG, AK8963_ASAX); // I2C slave 0 register address from where to begin data transfer
	writeByte(MPUnum, I2C_SLV0_CTRL, 0x83);       // Enable I2C and read 3 bytes
	HAL_Delay(50);
	readBytes(MPUnum, EXT_SENS_DATA_00, 3, &rawData[0]); // Read the x-, y-, and z-axis calibration values
	magCalibration[0] = (float) (rawData[0] - 128) / 256.0f + 1.0f; // Return x-axis sensitivity adjustment values, etc.
	magCalibration[1] = (float) (rawData[1] - 128) / 256.0f + 1.0f;
	magCalibration[2] = (float) (rawData[2] - 128) / 256.0f + 1.0f;

	dest1[0] = (float) mag_bias[0] * _mRes * magCalibration[0]; // save mag biases in G for main program
	dest1[1] = (float) mag_bias[1] * _mRes * magCalibration[1];
	dest1[2] = (float) mag_bias[2] * _mRes * magCalibration[2];

	// Get soft iron correction estimate
	mag_scale[0] = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
	mag_scale[1] = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
	mag_scale[2] = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	dest2[0] = avg_rad / ((float) mag_scale[0]);
	dest2[1] = avg_rad / ((float) mag_scale[1]);
	dest2[2] = avg_rad / ((float) mag_scale[2]);

	printf("Mag Calibration done!");
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(uint8_t MPUnum, float * dest1, float * dest2) {
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	// reset device
	writeByte(MPUnum, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	HAL_Delay(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	writeByte(MPUnum, PWR_MGMT_1, 0x01);
	writeByte(MPUnum, PWR_MGMT_2, 0x00);
	HAL_Delay(200);

// Configure device for bias calculation
	writeByte(MPUnum, INT_ENABLE, 0x00);   // Disable all interrupts
	writeByte(MPUnum, FIFO_EN, 0x00);      // Disable FIFO
	writeByte(MPUnum, PWR_MGMT_1, 0x00);   // Turn on internal clock source
	writeByte(MPUnum, I2C_MST_CTRL, 0x00); // Disable I2C master
	writeByte(MPUnum, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	writeByte(MPUnum, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	HAL_Delay(15);

// Configure MPU6050 gyro and accelerometer for bias calculation
	writeByte(MPUnum, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	writeByte(MPUnum, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	writeByte(MPUnum, GYRO_CONFIG, 0x00); // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeByte(MPUnum, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t gyrosensitivity = 131;   // = 131 LSB/degrees/sec
	uint16_t accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByte(MPUnum, USER_CTRL, 0x40);   // Enable FIFO
	writeByte(MPUnum, FIFO_EN, 0x78); // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	HAL_Delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
	writeByte(MPUnum, FIFO_EN, 0x00); // Disable gyro and accelerometer sensors for FIFO
	readBytes(MPUnum, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t) data[0] << 8) | data[1];
	packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
		readBytes(MPUnum, FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t) data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t) data[2] << 8) | data[3]);
		accel_temp[2] = (int16_t) (((int16_t) data[4] << 8) | data[5]);
		gyro_temp[0] = (int16_t) (((int16_t) data[6] << 8) | data[7]);
		gyro_temp[1] = (int16_t) (((int16_t) data[8] << 8) | data[9]);
		gyro_temp[2] = (int16_t) (((int16_t) data[10] << 8) | data[11]);

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0] += (int32_t) gyro_temp[0];
		gyro_bias[1] += (int32_t) gyro_temp[1];
		gyro_bias[2] += (int32_t) gyro_temp[2];

	}
	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0] /= (int32_t) packet_count;
	gyro_bias[1] /= (int32_t) packet_count;
	gyro_bias[2] /= (int32_t) packet_count;

	if (accel_bias[2] > 0L) {
		accel_bias[2] -= (int32_t) accelsensitivity;
	}  // Remove gravity from the z-axis accelerometer bias calculation
	else {
		accel_bias[2] += (int32_t) accelsensitivity;
	}

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
	data[3] = (-gyro_bias[1] / 4) & 0xFF;
	data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
	data[5] = (-gyro_bias[2] / 4) & 0xFF;

// Push gyro biases to hardware registers
	writeByte(MPUnum, XG_OFFSET_H, data[0]);
	writeByte(MPUnum, XG_OFFSET_L, data[1]);
	writeByte(MPUnum, YG_OFFSET_H, data[2]);
	writeByte(MPUnum, YG_OFFSET_L, data[3]);
	writeByte(MPUnum, ZG_OFFSET_H, data[4]);
	writeByte(MPUnum, ZG_OFFSET_L, data[5]);

// Output scaled gyro biases for display in the main program
	dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
	dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
	readBytes(MPUnum, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t) (((int16_t) data[0] << 8) | data[1]);
	readBytes(MPUnum, YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int32_t) (((int16_t) data[0] << 8) | data[1]);
	readBytes(MPUnum, ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int32_t) (((int16_t) data[0] << 8) | data[1]);

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = { 0, 0, 0 }; // Define array to hold mask bit for each accelerometer bias axis

	for (ii = 0; ii < 3; ii++) {
		if ((accel_bias_reg[ii] & mask))
			mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0]) & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1]) & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2]) & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
//  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
//  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
//  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
//  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
//  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
//  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

// Output scaled accelerometer biases for display in the main program
	dest2[0] = (float) accel_bias[0] / (float) accelsensitivity;
	dest2[1] = (float) accel_bias[1] / (float) accelsensitivity;
	dest2[2] = (float) accel_bias[2] / (float) accelsensitivity;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void SelfTest(uint8_t MPUnum, float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
	uint8_t rawData[6] = { 0, 0, 0, 0, 0, 0 };
	uint8_t selfTest[6];
	int32_t gAvg[3] = { 0 }, aAvg[3] = { 0 }, aSTAvg[3] = { 0 }, gSTAvg[3] =
			{ 0 };
	float factoryTrim[6];
	uint8_t FS = 0;

	writeByte(MPUnum, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
	writeByte(MPUnum, CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	writeByte(MPUnum, GYRO_CONFIG, 1 << FS); // Set full scale range for the gyro to 250 dps
	writeByte(MPUnum, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	writeByte(MPUnum, ACCEL_CONFIG, 1 << FS); // Set full scale range for the accelerometer to 2 g

	for (int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

		readBytes(MPUnum, ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
		aAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		aAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);

		readBytes(MPUnum, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		gAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
	}

	for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

// Configure the accelerometer for self-test
	writeByte(MPUnum, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	writeByte(MPUnum, GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	HAL_Delay(25);  // Delay a while to let the device stabilize

	for (int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

		readBytes(MPUnum, ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
		aSTAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		aSTAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);

		readBytes(MPUnum, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		gSTAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
	}

	for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	writeByte(MPUnum, ACCEL_CONFIG, 0x00);
	writeByte(MPUnum, GYRO_CONFIG, 0x00);
	HAL_Delay(25);  // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	selfTest[0] = readByte(MPUnum, SELF_TEST_X_ACCEL); // X-axis accel self-test results
	selfTest[1] = readByte(MPUnum, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
	selfTest[2] = readByte(MPUnum, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	selfTest[3] = readByte(MPUnum, SELF_TEST_X_GYRO); // X-axis gyro self-test results
	selfTest[4] = readByte(MPUnum, SELF_TEST_Y_GYRO); // Y-axis gyro self-test results
	selfTest[5] = readByte(MPUnum, SELF_TEST_Z_GYRO); // Z-axis gyro self-test results

	// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[2] - 1.0))); // FT[Za] factory trim calculation
	factoryTrim[3] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++) {
		destination[i] = 100.0f * ((float) (aSTAvg[i] - aAvg[i]))
				/ factoryTrim[i] - 100.0f;   // Report percent differences
		destination[i + 3] = 100.0f * ((float) (gSTAvg[i] - gAvg[i]))
				/ factoryTrim[i + 3] - 100.0f; // Report percent differences
	}

}

// I2C read/write functions for the MPU9250 sensors
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)

{
	uint8_t data_write[2];
	data_write[0] = subAddress;
	data_write[1] = data;

	HAL_I2C_Master_Transmit(&hi2c2, 0xD0, data_write, 2, 10);

}

uint8_t readByte(uint8_t address, uint8_t subAddress) {
	uint8_t data[1]; // `data` will store the register data
	uint8_t data_write[1];
	data_write[0] = subAddress;

	HAL_I2C_Master_Transmit(&hi2c2, 0xD0, data_write, 1, 10); //Send adress of register ONLY
	HAL_I2C_Master_Receive(&hi2c2, 0xD1, data, 1, 10);

	return data[0];

}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count,
		uint8_t * dest) {

	uint8_t data_write[1];
	data_write[0] = subAddress;

	HAL_I2C_Master_Transmit(&hi2c2, 0xD0, data_write, 1, 10);
	HAL_I2C_Master_Receive(&hi2c2, 0xD1, dest, count, 10);

}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
__attribute__((optimize("O3"))) void MadgwickQuaternionUpdate1(float ax,
		float ay, float az, float gx, float gy, float gz, float mx, float my,
		float mz) {
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrtf(ax * ax + ay * ay + az * az);
	if (norm == 0.0f)
		return; // handle NaN
	norm = 1.0f / norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrtf(mx * mx + my * my + mz * mz);
	if (norm == 0.0f)
		return; // handle NaN
	norm = 1.0f / norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3
			+ _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2
			+ my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrtf(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2
			+ _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax)
			+ _2q2 * (2.0f * q1q2 + _2q3q4 - ay)
			- _2bz * q3
					* (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
			+ (-_2bx * q4 + _2bz * q2)
					* (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
			+ _2bx * q3
					* (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay)
			- 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az)
			+ _2bz * q4
					* (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
			+ (_2bx * q3 + _2bz * q1)
					* (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
			+ (_2bx * q4 - _4bz * q2)
					* (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax)
			+ _2q4 * (2.0f * q1q2 + _2q3q4 - ay)
			- 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az)
			+ (-_4bx * q3 - _2bz * q1)
					* (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
			+ (_2bx * q2 + _2bz * q4)
					* (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
			+ (_2bx * q1 - _4bz * q3)
					* (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay)
			+ (-_4bx * q4 + _2bz * q2)
					* (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
			+ (-_2bx * q1 + _2bz * q3)
					* (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
			+ _2bx * q2
					* (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // normalise step magnitude
	norm = 1.0f / norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * deltat1;
	q2 += qDot2 * deltat1;
	q3 += qDot3 * deltat1;
	q4 += qDot4 * deltat1;
	norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); // normalise quaternion
	norm = 1.0f / norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Self test and calibrate, then init
void imu_init() {

	// Read the WHO_AM_I register, this is a good test of communication
	printf("MPU9250 9-axis motion sensor... \r\n");
	uint8_t c = getMPU9250ID(MPU1);

	if (c == 0x71) // WHO_AM_I should always be 0x71 for MPU9250, 0x73 for MPU9255
			{
		printf("MPU9250s 1 online...\r\n");

		resetMPU9250(MPU1); // start by resetting MPU9250_1

		SelfTest(MPU1, selfTest); // Start by performing self test and reporting values
		printf("Self Test for MPU9250 #1: \r\n");
		printf("x-axis self test: acceleration trim within : ");
		printf("%f ", selfTest[0]);
		printf("pct of factory value \n\r");
		printf("y-axis self test: acceleration trim within : ");
		printf("%f", selfTest[1]);
		printf("pct of factory value \n\r");
		printf("z-axis self test: acceleration trim within : ");
		printf("%f", selfTest[2]);
		printf("pct of factory value \n\r");
		printf("x-axis self test: gyration trim within : ");
		printf("%f", selfTest[3]);
		printf("pct of factory value \n\r");
		printf("y-axis self test: gyration trim within : ");
		printf("%f", selfTest[4]);
		printf("pct of factory value \n\r");
		printf("z-axis self test: gyration trim within : ");
		printf("%f", selfTest[5]);
		printf("pct of factory value \n\r");
	}

	HAL_Delay(1000);

	// get sensor resolutions, only need to do this once, same for both MPU9250s for now
	aRes = getAres(Ascale);
	gRes = getGres(Gscale);
	mRes = getMres(Mscale);

	// Comment out if using pre-measured, pre-stored offset biases
	calibrateMPU9250(MPU1, gyroBias1, accelBias1); // Calibrate gyro and accelerometers, load biases in bias registers
	printf("MPU1 accel biases (mg)\n\r");
	printf("%f \n\r", 1000. * accelBias1[0]);
	printf("%f \n\r", 1000. * accelBias1[1]);
	printf("%f \n\r", 1000. * accelBias1[2]);
	printf("MPU1 gyro biases (dps)\n\r");
	printf("%f \n\r", gyroBias1[0]);
	printf("%f \n\r", gyroBias1[1]);
	printf("%f \n\r", gyroBias1[2]);

	initMPU9250(MPU1, Ascale, Gscale, sampleRate);
	printf("MPU9250 Initialised\n\r");

	// Read the WHO_AM_I register of the magnetometer, this is a good test of communication
	uint8_t e = getAK8963CID(MPU1);  // Read WHO_AM_I register for AK8963
	printf("AK8963 1 \n\r");
	printf("I AM \n\r");
	printf("%x \n\r", e);
	printf(" I should be \n\r");
	printf("%x", 0x48);

	HAL_Delay(1000);

	// Get magnetometer calibration from AK8963 ROM
	initAK8963Slave(MPU1, Mscale, Mmode, magCalibration1);
	printf("AK8963 1 initialized for active data mode...."); // Initialize device 1 for active mode read of magnetometer
	printf("Calibration values for mag 1: ");
	printf("X-Axis sensitivity adjustment value \n\r");
	printf("%f \n\r", magCalibration1[0]);
	printf("Y-Axis sensitivity adjustment value \n\r");
	printf("%f \n\r", magCalibration1[1]);
	printf("Z-Axis sensitivity adjustment value \n\r");
	printf("%f \n\r", magCalibration1[2]);

	// Comment out if using pre-measured, pre-stored offset biases
	magcalMPU9250(MPU1, magBias1, magScale1);
	printf("AK8963 1 mag biases (mG)\n\r");
	printf("%f \n\r", magBias1[0]);
	printf("%f \n\r", magBias1[1]);
	printf("%f \n\r", magBias1[2]);
	printf("AK8963 1 mag scale (mG)\n\r");
	printf("%f \n\r", magScale1[0]);
	printf("%f \n\r", magScale1[1]);
	printf("%f \n\r", magScale1[2]);

	HAL_Delay(2000);

	printf(" Calibration complete! \n\r");

}

#define TIMER_MAX_LOAD 65535.0f
#define TIMER_CLK_FREQ 100000000.0f
#define TIMER_PRESCALER 100.0f

void calculate_euler(int tim_counter_value) {


	// If intPin1 goes high, either all data registers have new data
   // On interrupt, read data
	      intFlag1 = false;     // reset newData flag

	      if (readByte(0, INT_STATUS) & 0x01){

	    readMPU9250Data(MPU1, MPU9250Data1); // INT cleared on any read

	    // Now we'll calculate the accleration value into actual g's
	     ax1 = (float)MPU9250Data1[0]*aRes - accelBias1[0];  // get actual g value, this depends on scale being set
	     ay1 = (float)MPU9250Data1[1]*aRes - accelBias1[1];
	     az1 = (float)MPU9250Data1[2]*aRes - accelBias1[2];

	    // Calculate the gyro value into actual degrees per second
	     gx1 = (float)MPU9250Data1[4]*gRes;  // get actual gyro value, this depends on scale being set
	     gy1 = (float)MPU9250Data1[5]*gRes;
	     gz1 = (float)MPU9250Data1[6]*gRes;

	  // if( checkNewMagData(MPU1) == true) { // wait for magnetometer data ready bit to be set
	      readMagData(MPU1, magCount1);  // Read the x/y/z adc values

	    // Calculate the magnetometer values in milliGauss
	    // Include factory calibration per data sheet and user environmental corrections
	      mx1 = (float)magCount1[0]*mRes*magCalibration1[0] - magBias1[0];  // get actual magnetometer value, this depends on scale being set
	      my1 = (float)magCount1[1]*mRes*magCalibration1[1] - magBias1[1];
	      mz1 = (float)magCount1[2]*mRes*magCalibration1[2] - magBias1[2];
	      mx1 *= magScale1[0];
	      my1 *= magScale1[1];
	      mz1 *= magScale1[2];
	 //}


	    for(uint8_t i = 0; i < 10; i++) { // iterate a fixed number of times per data read cycle


	    Now1 = tim_counter_value;

		//This will happen when the timer takes a reading, then the next reading is after
		//the timer has reset, ie gone back to 0 and started counting up again
		if(Now1 - lastUpdate1 < 0){
			//Take time difference taking into account reset of timer
			//Formula for getting timer count into seconds = COUNT * (1/TIMER_CLK)*PRESCALER
			deltat1 = (float) (((TIMER_MAX_LOAD-lastUpdate1)+Now1) * (1 / (TIMER_CLK_FREQ / TIMER_PRESCALER)));
		}
		//Otherwise normally the count difference will be positive
		deltat1 = (float) ((Now1 - lastUpdate1) * (1 / (TIMER_CLK_FREQ / TIMER_PRESCALER))); // set integration time by time elapsed since last filter update

	   // deltat1 = ((Now1 - lastUpdate1)/1000000.0f); // set integration time by time elapsed since last filter update

	    lastUpdate1 = Now1;

	    sum1 += deltat1; // sum for averaging filter update rate
	    sumCount1++;

	    MadgwickQuaternionUpdate1(-ax1, +ay1, +az1, gx1*pi/180.0f, -gy1*pi/180.0f, -gz1*pi/180.0f,  my1,  -mx1, mz1);
	    }




	    a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
	    a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
	    a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
	    a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
	    a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
	    pitch1 = -asinf(a32);
	    roll1  = atan2f(a31, a33);
	    yaw1   = atan2f(a12, a22);
	    pitch1 *= 180.0f / pi;
	    yaw1   *= 180.0f / pi;
	    yaw1   += -0.27f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
	    if(yaw1 < 0) yaw1   += 360.0f; // Ensure yaw stays between 0 and 360
	    roll1  *= 180.0f / pi;
	    lin_ax1 = ax1 + a31;
	    lin_ay1 = ay1 + a32;
	    lin_az1 = az1 - a33;
	      }
	    /* end of MPU9250 1 interrupt handling */

}
