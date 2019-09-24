/*
 * IMU.c
 *
 *  Created on: Jun 19, 2019
 *      Author: nick_savva
 */

#include "IMU.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "tim.h"

//Timer clock frequency, in this case same as MCU frequency, 100MHz
#define TIMER_CLK_FREQ 100000000.0f

I2C_HandleTypeDef hi2c2;

float sum = 0;
uint32_t sumCount = 0, mcount = 0;

// Set initial input parameters
enum Ascale {
	AFS_2G = 0, AFS_4G, AFS_8G, AFS_16G
};

enum Gscale {
	GFS_250DPS = 0, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
};

enum Mscale {
	MFS_14BITS = 0, // 0.6 mG per LSB
	MFS_16BITS      // 0.15 mG per LSB
};

uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
uint8_t Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06; // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = { 1.1953125, 1.19921875, 1.15234375 }, magbias[3] = { -35.4058418, -98.8425751, 654.464783 }, magscale[3] = {1,1,1};// Factory mag calibration and mag bias
float gyroBias[3] = { 0.435114503, -0.167938933, -0.587786257 }, accelBias[3] = { 0.005859375, 0.0134887695, -0.0504150391}; // Bias corrections for gyro and accelerometer
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
int16_t tempCount; // Stores the real internal chip temperature in degrees Celsius
float temperature;
float SelfTest[6];

int delt_t = 0; // used to control display output rate
int count = 0;  // used to control display output rate

// parameters for 6 DoF sensor fusion calculations
//#define PI 3.14159265358979323846 //PI DEFINED IN arm_math.h
float GyroMeasError = PI * (60.0f / 180.0f); // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * PI * (60.0f / 180.0f);  // compute beta
float GyroMeasDrift = PI * (1.0f / 180.0f); // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * PI * (1.0f / 180.0f); // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

float pitch, yaw, roll;
float deltat = 0.0f;             // integration interval for both filter schemes
float32_t lastUpdate = 0, firstUpdate = 0, Now = 0; // used to calculate integration interval                               // used to calculate integration interval
float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };           // vector to hold quaternion
float eInt[3] = { 0.0f, 0.0f, 0.0f }; // vector to hold integral error for Mahony method




/***************************** METHODS  **********************************/

IMU_StatusTypeDef imu_init(I2C_HandleTypeDef* handle) {

	//printf("STM32 Online..\r\n");

	//Check if MPU9250 responds
	if (HAL_I2C_IsDeviceReady(handle, 0xD0, 2, 100) == HAL_OK) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); //Toggle LED on if so
	}
	else{
		while(1);
	}

	uint8_t whoami = readByte(MPU9250_ADDRESS_TX, MPU9250_ADDRESS_RX,
	WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU9250

	if (whoami == 0x71) {
		printf("MPU9250 online..\r\n");
		printf("whoami reg = %d ..\r\n", whoami);

		//uint8_t UART_TX_DATA[13] = "MPU9250 online..\r\n";
		//HA L_UART_Transmit(&huart2, UART_TX_DATA, 13, 10);
	}


	return IMU_SUCCESS;

}

IMU_StatusTypeDef imu_calibrate() {

	//************** Calibration sequence for MPU9250 **************
	//This is mostly taken from the library example https://os.mbed.com/users/onehorse/code/MPU9250AHRS/file/4e59a37182df/main.cpp/
	//but adapted slightly for this platform

	resetMPU9250();
	MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values

	HAL_Delay(500);



	//calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

	HAL_Delay(500);

	initMPU9250();
	initAK8963(magCalibration);


	HAL_Delay(500);

	getAres(); // Get accelerometer sensitivity
	getGres(); // Get gyro sensitivity
	getMres(); // Get magnetometer sensitivity

	//magcalMPU9250(magbias);

//	magbias[0] = +470.; // User environmental x-axis correction in milliGauss, should be automatically calculated
//	magbias[1] = +120.; // User environmental x-axis correction in milliGauss
//	magbias[2] = +125.; // User environmental x-axis correction in milliGauss

	return IMU_SUCCESS;

}

void calc_RollPitchYaw(float *roll, float *pitch, float *yaw) {

	//******* Get roll pitch & yaw values from registers (also from library example but adapted slightly) ********

	// If intPin goes high, all data registers have new data
	if (readByte(MPU9250_ADDRESS_TX, MPU9250_ADDRESS_RX, INT_STATUS) & 0x01) { // On interrupt, check if data ready interrupt

		readAccelData(accelCount);  // Read the x/y/z adc values
		// Now we'll calculate the accleration value into actual g's
		ax = (float) accelCount[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
		ay = (float) accelCount[1] * aRes - accelBias[1];
		az = (float) accelCount[2] * aRes - accelBias[2];

		readGyroData(gyroCount);  // Read the x/y/z adc values
		// Calculate the gyro value into actual degrees per second
		gx = (float) gyroCount[0] * gRes - gyroBias[0]; // get actual gyro value, this depends on scale being set
		gy = (float) gyroCount[1] * gRes - gyroBias[1];
		gz = (float) gyroCount[2] * gRes - gyroBias[2];

		readMagData(magCount);  // Read the x/y/z adc values
		// Calculate the magnetometer values in milliGauss
		// Include factory calibration per data sheet and user environmental corrections
		mx = (float) magCount[0] * mRes * magCalibration[0] - magbias[0]; // get actual magnetometer value, this depends on scale being set
		my = (float) magCount[1] * mRes * magCalibration[1] - magbias[1];
		mz = (float) magCount[2] * mRes * magCalibration[2] - magbias[2];

		mx *= magscale[0];
		my *= magscale[1];
		mz *= magscale[2];
	}

	Now = htim11.Instance->CNT;

	//This will happen when the timer takes a reading, then the next reading is after
	//the timer has reset, ie gone back to 0 and started counting up again
	if(Now - lastUpdate < 0){
		//Take time difference taking into account reset of timer
		//Formula for getting timer count into seconds = COUNT * (1/TIMER_CLK)*PRESCALER
		deltat = (float) (((65535-lastUpdate)+Now) * (1 / (TIMER_CLK_FREQ / 100.0f)));

	}
	else{
	//Otherwise normally the count difference will be positive
	deltat = (float) ((Now - lastUpdate) * (1 / (TIMER_CLK_FREQ / 100.0f))); // set integration time by time elapsed since last filter update

	}

	lastUpdate = Now;


	sum += deltat;
	sumCount++;

	// Pass gyro rate as rad/s
	MahonyQuaternionUpdate(ax, ay, az, gx * PI / 180.0, gy * PI / 180.0, gz * PI / 180.0, my, mx, mz);


	// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
	// In this coordinate system, the positive z-axis is down toward Earth.
	// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
	// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
	// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
	// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
	// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
	// applied in the correct order which for this configuration is yaw, pitch, and then roll.
	// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
	*yaw = atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
	*pitch = -asin(2.0 * (q[1] * q[3] - q[0] * q[2]));
	*roll = atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
	*pitch *= 180.0 / PI;
	*yaw *= 180.0 / PI;
	*yaw -=  -1.1; // CHANGE-> (In Leeds, UK declination = -1.1) ... Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04 (+13.8)
	*roll *= 180.0 / PI;

	sum = 0;
	sumCount = 0;

}


//===================================================================================================================
//====== Set of useful function to access acceleration, gyroscope, and temperature data
//===================================================================================================================

void writeByte(uint8_t address_tx, uint8_t subAddress, uint8_t data) {
	uint8_t data_write[2];
	data_write[0] = subAddress;
	data_write[1] = data;
	//i2c.write(address, data_write, 2, 0);


	HAL_I2C_Master_Transmit(&hi2c2, address_tx, data_write, 2, 10);
}

//NICK - I've changed these to accept a tx address & a rx address as STM32 boards include the R/W bit at the end of 7 bit adress
//so HAL function takes in 8 bit address including R/W bit
char readByte(uint8_t address_tx, uint8_t address_rx, uint8_t subAddress) {
	uint8_t data[1]; // `data` will store the register data
	uint8_t data_write[1];
	data_write[0] = subAddress;
	//i2c.write(address, data_write, 1, 1); // no stop
	//i2c.read(address, data, 1, 0);


	HAL_I2C_Master_Transmit(&hi2c2, address_tx, data_write, 1, 10); //Send adress of register ONLY
	HAL_I2C_Master_Receive(&hi2c2, address_tx, data, 1, 10);

	return data[0];
}

void readBytes(uint8_t address_tx, uint8_t address_rx, uint8_t subAddress,
	uint8_t count, uint8_t * dest) {
	uint8_t data[14];
	uint8_t data_write[1];
	data_write[0] = subAddress;
	//i2c.write(address, data_write, 1, 1); // no stop
	//i2c.read(address, data, count, 0);


	HAL_I2C_Master_Transmit(&hi2c2, address_tx, data_write, 1, 10);
	HAL_I2C_Master_Receive(&hi2c2, address_rx, data, count, 10);

	for (int ii = 0; ii < count; ii++) {
		dest[ii] = data[ii];
	}
}

void getMres() {

	switch (Mscale) {
	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
	case MFS_14BITS:
		mRes = 10.0 * 4219.0 / 8190.0; // Proper scale to return milliGauss
		break;
	case MFS_16BITS:
		mRes = 10.0 * 4219.0 / 32760.0; // Proper scale to return milliGauss
		break;
	}
}

void getGres() {
	switch (Gscale) {
	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
	// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case GFS_250DPS:
		gRes = 250.0 / 32768.0;
		break;
	case GFS_500DPS:
		gRes = 500.0 / 32768.0;
		break;
	case GFS_1000DPS:
		gRes = 1000.0 / 32768.0;
		break;
	case GFS_2000DPS:
		gRes = 2000.0 / 32768.0;
		break;
	}
}

void getAres() {
	switch (Ascale) {
	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
	// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case AFS_2G:
		aRes = 2.0 / 32768.0;
		break;
	case AFS_4G:
		aRes = 4.0 / 32768.0;
		break;
	case AFS_8G:
		aRes = 8.0 / 32768.0;
		break;
	case AFS_16G:
		aRes = 16.0 / 32768.0;
		break;
	}
}

void readAccelData(int16_t * destination) {
	uint8_t rawData[6];  // x/y/z accel register data stored here
	readBytes(MPU9250_ADDRESS_TX, MPU9250_ADDRESS_RX, ACCEL_XOUT_H, 6,
			&rawData[0]); // Read the six raw data registers into data array
	destination[0] = (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
	destination[2] = (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
}

void readGyroData(int16_t * destination) {
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(MPU9250_ADDRESS_TX, MPU9250_ADDRESS_RX, GYRO_XOUT_H, 6,
			&rawData[0]); // Read the six raw data registers sequentially into data array
	destination[0] = (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
	destination[2] = (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
}

void readMagData(int16_t * destination) {
	uint8_t rawData[7]; // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	if (readByte(AK8963_ADDRESS_TX, AK8963_ADDRESS_RX, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set

		readBytes(AK8963_ADDRESS_TX, AK8963_ADDRESS_RX, AK8963_XOUT_L, 7, &rawData[0]); // Read the six raw data and ST2 registers sequentially into data array
		uint8_t c = rawData[6]; // End data read by reading ST2 register
		if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
			destination[0] =
					(int16_t) (((int16_t) rawData[1] << 8) | rawData[0]); // Turn the MSB and LSB into a signed 16-bit value
			destination[1] =
					(int16_t) (((int16_t) rawData[3] << 8) | rawData[2]); // Data stored as little Endian
			destination[2] =
					(int16_t) (((int16_t) rawData[5] << 8) | rawData[4]);
		}
	}
}

int16_t readTempData() {
	uint8_t rawData[2];  // x/y/z gyro register data stored here
	readBytes(MPU9250_ADDRESS_TX, MPU9250_ADDRESS_RX, TEMP_OUT_H, 2,
			&rawData[0]); // Read the two raw data registers sequentially into data array
	return (int16_t) (((int16_t) rawData[0]) << 8 | rawData[1]); // Turn the MSB and LSB into a 16-bit value
}

void resetMPU9250() {
	// reset device
	writeByte(MPU9250_ADDRESS_TX, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	HAL_Delay(100);
}

void initAK8963(float * destination) {
	// First extract the factory calibration for each magnetometer axis
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	writeByte(AK8963_ADDRESS_TX, AK8963_CNTL, 0x00); // Power down magnetometer
	HAL_Delay(10);
	writeByte(AK8963_ADDRESS_TX, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	HAL_Delay(10);
	readBytes(AK8963_ADDRESS_TX, AK8963_ADDRESS_RX, AK8963_ASAX, 3,
			&rawData[0]);  // Read the x-, y-, and z-axis calibration values
	destination[0] = (float) (rawData[0] - 128) / 256.0f + 1.0f; // Return x-axis sensitivity adjustment values, etc.
	destination[1] = (float) (rawData[1] - 128) / 256.0f + 1.0f;
	destination[2] = (float) (rawData[2] - 128) / 256.0f + 1.0f;
	writeByte(AK8963_ADDRESS_TX, AK8963_CNTL, 0x00); // Power down magnetometer
	HAL_Delay(10);
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	writeByte(AK8963_ADDRESS_TX, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
	HAL_Delay(10);
}


void magcalMPU9250(float * dest1) {
	uint16_t ii = 0, sample_count = 0;
	  int32_t mag_bias[3] = {0, 0, 0};
	  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};


	  HAL_Delay(200);

	   sample_count = 1500;
	   for(ii = 0; ii < sample_count; ii++) {
	    readMagData(mag_temp);  // Read the mag data
	    for (int jj = 0; jj < 3; jj++) {
	      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
	      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
	    }
	    HAL_Delay(10);  // at 8 Hz ODR, new mag data is available every 125 ms
	   }


	    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
	    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
	    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

	    dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
	    dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];
	    dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];

}

void initMPU9250() {
	// Initialize MPU9250 device
	// wake up device
	writeByte(MPU9250_ADDRESS_TX, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
	HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

	// get stable time source
	writeByte(MPU9250_ADDRESS_TX, PWR_MGMT_1, 0x01); // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

	// Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
	// DLPF_CFG = bits 2:0 = 010 = 0x03; this sets the sample rate at 1 kHz for both
	// Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
	writeByte(MPU9250_ADDRESS_TX, CONFIG, 0x00);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	writeByte(MPU9250_ADDRESS_TX, SMPLRT_DIV, 0x00); // Use a 200 Hz rate; the same rate set in CONFIG above

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = readByte(MPU9250_ADDRESS_RX, MPU9250_ADDRESS_RX,
	GYRO_CONFIG);
	writeByte(MPU9250_ADDRESS_TX, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	writeByte(MPU9250_ADDRESS_TX, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	writeByte(MPU9250_ADDRESS_TX, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro

	// Set accelerometer configuration
	c = readByte(MPU9250_ADDRESS_RX, MPU9250_ADDRESS_RX, ACCEL_CONFIG);
	writeByte(MPU9250_ADDRESS_TX, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	writeByte(MPU9250_ADDRESS_TX, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	writeByte(MPU9250_ADDRESS_TX, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = readByte(MPU9250_ADDRESS_RX, MPU9250_ADDRESS_RX, ACCEL_CONFIG2);
	writeByte(MPU9250_ADDRESS_TX, ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	writeByte(MPU9250_ADDRESS_TX, ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	writeByte(MPU9250_ADDRESS_TX, INT_PIN_CFG, 0x22);
	writeByte(MPU9250_ADDRESS_TX, INT_ENABLE, 0x01); // Enable data ready (bit 0) interrupt
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(float * dest1, float * dest2) {
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	// reset device, reset all registers, clear gyro and accelerometer bias registers
	writeByte(MPU9250_ADDRESS_TX, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	HAL_Delay(100);

	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	writeByte(MPU9250_ADDRESS_TX, PWR_MGMT_1, 0x01);
	writeByte(MPU9250_ADDRESS_TX, PWR_MGMT_2, 0x00);
	HAL_Delay(200);

	// Configure device for bias calculation
	writeByte(MPU9250_ADDRESS_TX, INT_ENABLE, 0x00); // Disable all interrupts
	writeByte(MPU9250_ADDRESS_TX, FIFO_EN, 0x00);      // Disable FIFO
	writeByte(MPU9250_ADDRESS_TX, PWR_MGMT_1, 0x00); // Turn on internal clock source
	writeByte(MPU9250_ADDRESS_TX, I2C_MST_CTRL, 0x00); // Disable I2C master
	writeByte(MPU9250_ADDRESS_TX, USER_CTRL, 0x00); // Disable FIFO and I2C master modes
	writeByte(MPU9250_ADDRESS_TX, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	HAL_Delay(15);

	// Configure MPU9250 gyro and accelerometer for bias calculation
	writeByte(MPU9250_ADDRESS_TX, CONFIG, 0x01); // Set low-pass filter to 188 Hz
	writeByte(MPU9250_ADDRESS_TX, SMPLRT_DIV, 0x00); // Set sample rate to 1 kHz
	writeByte(MPU9250_ADDRESS_TX, GYRO_CONFIG, 0x00); // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeByte(MPU9250_ADDRESS_TX, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t gyrosensitivity = 131;   // = 131 LSB/degrees/sec
	uint16_t accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByte(MPU9250_ADDRESS_TX, USER_CTRL, 0x40);   // Enable FIFO
	writeByte(MPU9250_ADDRESS_TX, FIFO_EN, 0x78); // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
	HAL_Delay(40); // accumulate 40 samples in 80 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	writeByte(MPU9250_ADDRESS_TX, FIFO_EN, 0x00); // Disable gyro and accelerometer sensors for FIFO
	readBytes(MPU9250_ADDRESS_TX, MPU9250_ADDRESS_RX, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t) data[0] << 8) | data[1];
	packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
		readBytes(MPU9250_ADDRESS_TX, MPU9250_ADDRESS_RX, FIFO_R_W, 12,
				&data[0]); // read data for averaging
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

	/// Push gyro biases to hardware registers
	/*  writeByte(MPU9250_ADDRESS_TX, XG_OFFSET_H, data[0]);
	 writeByte(MPU9250_ADDRESS_TX, XG_OFFSET_L, data[1]);
	 writeByte(MPU9250_ADDRESS_TX, YG_OFFSET_H, data[2]);
	 writeByte(MPU9250_ADDRESS_TX, YG_OFFSET_L, data[3]);
	 writeByte(MPU9250_ADDRESS_TX, ZG_OFFSET_H, data[4]);
	 writeByte(MPU9250_ADDRESS_TX, ZG_OFFSET_L, data[5]);
	 */
	dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
	dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
	readBytes(MPU9250_ADDRESS_TX, MPU9250_ADDRESS_RX, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int16_t) ((int16_t) data[0] << 8) | data[1];
	readBytes(MPU9250_ADDRESS_TX, MPU9250_ADDRESS_RX, YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int16_t) ((int16_t) data[0] << 8) | data[1];
	readBytes(MPU9250_ADDRESS_TX, MPU9250_ADDRESS_RX, ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int16_t) ((int16_t) data[0] << 8) | data[1];

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = { 0, 0, 0 }; // Define array to hold mask bit for each accelerometer bias axis

	for (ii = 0; ii < 3; ii++) {
		if (accel_bias_reg[ii] & mask)
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
	/*  writeByte(MPU9250_ADDRESS_TX, XA_OFFSET_H, data[0]);
	 writeByte(MPU9250_ADDRESS_TX, XA_OFFSET_L, data[1]);
	 writeByte(MPU9250_ADDRESS_TX, YA_OFFSET_H, data[2]);
	 writeByte(MPU9250_ADDRESS_TX, YA_OFFSET_L, data[3]);
	 writeByte(MPU9250_ADDRESS_TX, ZA_OFFSET_H, data[4]);
	 writeByte(MPU9250_ADDRESS_TX, ZA_OFFSET_L, data[5]);
	 */
	// Output scaled accelerometer biases for manual subtraction in the main program
	dest2[0] = (float) accel_bias[0] / (float) accelsensitivity;
	dest2[1] = (float) accel_bias[1] / (float) accelsensitivity;
	dest2[2] = (float) accel_bias[2] / (float) accelsensitivity;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
		{
	uint8_t rawData[6] = { 0, 0, 0, 0, 0, 0 };
	uint8_t selfTest[6];
	int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
	float factoryTrim[6];
	uint8_t FS = 0;

	writeByte(MPU9250_ADDRESS_TX, SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz
	writeByte(MPU9250_ADDRESS_TX, CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	writeByte(MPU9250_ADDRESS_TX, GYRO_CONFIG, 1 << FS); // Set full scale range for the gyro to 250 dps
	writeByte(MPU9250_ADDRESS_TX, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	writeByte(MPU9250_ADDRESS_TX, ACCEL_CONFIG, 1 << FS); // Set full scale range for the accelerometer to 2 g

	for (int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

		readBytes(MPU9250_ADDRESS_TX, MPU9250_ADDRESS_RX, ACCEL_XOUT_H, 6,
				&rawData[0]); // Read the six raw data registers into data array
		aAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		aAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);

		readBytes(MPU9250_ADDRESS_TX, MPU9250_ADDRESS_RX, GYRO_XOUT_H, 6,
				&rawData[0]); // Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		gAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
	}

	for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	// Configure the accelerometer for self-test
	writeByte(MPU9250_ADDRESS_TX, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	writeByte(MPU9250_ADDRESS_TX, GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	HAL_Delay(25); // Delay a while to let the device stabilize

	for (int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

		readBytes(MPU9250_ADDRESS_TX, MPU9250_ADDRESS_RX, ACCEL_XOUT_H, 6,
				&rawData[0]); // Read the six raw data registers into data array
		aSTAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		aSTAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);

		readBytes(MPU9250_ADDRESS_TX, MPU9250_ADDRESS_RX, GYRO_XOUT_H, 6,
				&rawData[0]); // Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		gSTAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
	}

	for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	writeByte(MPU9250_ADDRESS_TX, ACCEL_CONFIG, 0x00);
	writeByte(MPU9250_ADDRESS_TX, GYRO_CONFIG, 0x00);
	HAL_Delay(25); // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	selfTest[0] = readByte(MPU9250_ADDRESS_TX, MPU9250_ADDRESS_RX,
	SELF_TEST_X_ACCEL); // X-axis accel self-test results
	selfTest[1] = readByte(MPU9250_ADDRESS_TX, MPU9250_ADDRESS_RX,
	SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
	selfTest[2] = readByte(MPU9250_ADDRESS_TX, MPU9250_ADDRESS_RX,
	SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	selfTest[3] = readByte(MPU9250_ADDRESS_TX, MPU9250_ADDRESS_RX,
	SELF_TEST_X_GYRO); // X-axis gyro self-test results
	selfTest[4] = readByte(MPU9250_ADDRESS_TX, MPU9250_ADDRESS_RX,
	SELF_TEST_Y_GYRO); // Y-axis gyro self-test results
	selfTest[5] = readByte(MPU9250_ADDRESS_TX, MPU9250_ADDRESS_RX,
	SELF_TEST_Z_GYRO); // Z-axis gyro self-test results

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
		destination[i] = 100.0 * ((float) (aSTAvg[i] - aAvg[i]))
				/ factoryTrim[i]; // Report percent differences
		destination[i + 3] = 100.0 * ((float) (gSTAvg[i] - gAvg[i]))
				/ factoryTrim[i + 3]; // Report percent differences
	}

}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
		float gz, float mx, float my, float mz) {

	//Original Function starts here
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
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f)
		return; // handle NaN
	norm = 1.0f / norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
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
	_2bx = sqrt(hx * hx + hy * hy);
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
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // normalise step magnitude
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
	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); // normalise quaternion
	norm = 1.0f / norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;

}

// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
// measured ones.
__attribute__((optimize("-Ofast"))) void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
		float gz, float mx, float my, float mz) {

	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; // short name local variable for readability
	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;

	// Auxiliary variables to avoid repeated arithmetic
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
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f)
		return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f)
		return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4)
			+ 2.0f * mz * (q2q4 + q1q3);
	hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4)
			+ 2.0f * mz * (q3q4 - q1q2);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2)
			+ 2.0f * mz * (0.5f - q2q2 - q3q3);

	// Estimated direction of gravity and magnetic field
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	if (Ki > 0.0f) {
		eInt[0] += ex;      // accumulate integral error
		eInt[1] += ey;
		eInt[2] += ez;
	} else {
		eInt[0] = 0.0f;     // prevent integral wind up
		eInt[1] = 0.0f;
		eInt[2] = 0.0f;
	}

	// Apply feedback terms
	gx = gx + Kp * ex + Ki * eInt[0];
	gy = gy + Kp * ey + Ki * eInt[1];
	gz = gz + Kp * ez + Ki * eInt[2];

	// Integrate rate of change of quaternion
	pa = q2;
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
	q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
	q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
	q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

	// Normalise quaternion
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	norm = 1.0f / norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;

}
