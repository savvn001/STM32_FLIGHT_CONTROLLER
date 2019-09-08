/*
 * MPU9250_DMP.c
 *
 *  Created on: Sep 7, 2019
 *      Author: nick_savva
 */

/*
 * 	Translation of Sparkfun Arduino MPU9250 DMP library for an STM32F4 using HAL and CubeMX
 *
 *  https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library
 *
 */

#include "MPU9250_DMP.h"
#include "util/inv_mpu.h"
#include "util/MPU9250_RegisterMap.h"


const signed char defaultOrientation[9] = {
	1, 0, 0,
	0, 1, 0,
	0, 0, 1
};

enum t_axisOrder {
	X_AXIS, // 0
	Y_AXIS, // 1
	Z_AXIS  // 2
};

int ax, ay, az;
int gx, gy, gz;
int mx, my, mz;
long qw, qx, qy, qz;
long temperature;
unsigned long time;
float pitch, roll, yaw;
float heading;

unsigned short _aSense;
float _gSense, _mSense;

static unsigned char mpu9250_orientation;
static unsigned char tap_count;
static unsigned char tap_direction;
static bool _tap_available;
static void orient_cb(unsigned char orient);
static void tap_cb(unsigned char direction, unsigned char count);

long gyro[3], accel[3];

float imu_get_roll(){
return roll;
}
float imu_get_pitch(){
return pitch;
}
float imu_get_yaw(){
return yaw;
}


void MPU9250_DMP()
{
	_mSense = 6.665f; // Constant - 4915 / 32760
	_aSense = 0.0f;   // Updated after accel FSR is set
	_gSense = 0.0f;   // Updated after gyro FSR is set
}

inv_error_t imu_begin(void)
{
	inv_error_t result;
    struct int_param_s int_param;

	result = mpu_init(&int_param);

	if (result)
		return result;

	mpu_set_bypass(1); // Place all slaves (including compass) on primary bus

	imu_setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

	_gSense = imu_getGyroSens();
	_aSense = imu_getAccelSens();

	return result;
}

inv_error_t imu_enableInterrupt(unsigned char enable)
{
	return set_int_enable(enable);
}

inv_error_t imu_setIntLevel(unsigned char active_low)
{
	return mpu_set_int_level(active_low);
}

inv_error_t imu_setIntLatched(unsigned char enable)
{
	return mpu_set_int_latched(enable);
}

short imu_getIntStatus(void)
{
	short status;
	if (mpu_get_int_status(&status) == INV_SUCCESS)
	{
		return status;
	}
	return 0;
}

// Accelerometer Low-Power Mode. Rate options:
// 1.25 (1), 2.5 (2), 5, 10, 20, 40,
// 80, 160, 320, or 640 Hz
// Disables compass and gyro
inv_error_t imu_lowPowerAccel(unsigned short rate)
{
	return mpu_lp_accel_mode(rate);
}

inv_error_t imu_setGyroFSR(unsigned short fsr)
{
	inv_error_t err;
	err = mpu_set_gyro_fsr(fsr);
	if (err == INV_SUCCESS)
	{
		_gSense = imu_getGyroSens();
	}
	return err;
}

inv_error_t imu_setAccelFSR(unsigned char fsr)
{
	inv_error_t err;
	err = mpu_set_accel_fsr(fsr);
	if (err == INV_SUCCESS)
	{
		_aSense = imu_getAccelSens();
	}
	return err;
}

unsigned short imu_getGyroFSR(void)
{
	unsigned short tmp;
	if (mpu_get_gyro_fsr(&tmp) == INV_SUCCESS)
	{
		return tmp;
	}
	return 0;
}

unsigned char imu_getAccelFSR(void)
{
	unsigned char tmp;
	if (mpu_get_accel_fsr(&tmp) == INV_SUCCESS)
	{
		return tmp;
	}
	return 0;
}

unsigned short imu_getMagFSR(void)
{
	unsigned short tmp;
	if (mpu_get_compass_fsr(&tmp) == INV_SUCCESS)
	{
		return tmp;
	}
	return 0;
}

inv_error_t imu_setLPF(unsigned short lpf)
{
	return mpu_set_lpf(lpf);
}

unsigned short imu_getLPF(void)
{
	unsigned short tmp;
	if (mpu_get_lpf(&tmp) == INV_SUCCESS)
	{
		return tmp;
	}
	return 0;
}

inv_error_t imu_setSampleRate(unsigned short rate)
{
    return mpu_set_sample_rate(rate);
}

unsigned short imu_getSampleRate(void)
{
	unsigned short tmp;
	if (mpu_get_sample_rate(&tmp) == INV_SUCCESS)
	{
		return tmp;
	}
	return 0;
}

inv_error_t imu_setCompassSampleRate(unsigned short rate)
{
	return mpu_set_compass_sample_rate(rate);
}

unsigned short imu_getCompassSampleRate(void)
{
	unsigned short tmp;
	if (mpu_get_compass_sample_rate(&tmp) == INV_SUCCESS)
	{
		return tmp;
	}

	return 0;
}

float imu_getGyroSens(void)
{
	float sens;
	if (mpu_get_gyro_sens(&sens) == INV_SUCCESS)
	{
		return sens;
	}
	return 0;
}

unsigned short imu_getAccelSens(void)
{
	unsigned short sens;
	if (mpu_get_accel_sens(&sens) == INV_SUCCESS)
	{
		return sens;
	}
	return 0;
}

float getMagSens(void)
{
	return 0.15; // Static, 4915/32760
}

unsigned char imu_getFifoConfig(void)
{
	unsigned char sensors;
	if (mpu_get_fifo_config(&sensors) == INV_SUCCESS)
	{
		return sensors;
	}
	return 0;
}

inv_error_t imu_configureFifo(unsigned char sensors)
{
	return mpu_configure_fifo(sensors);
}

inv_error_t imu_resetFifo(void)
{
	return mpu_reset_fifo();
}

unsigned short imu_fifoAvailable(void)
{
	unsigned char fifoH, fifoL;

	if (mpu_read_reg(MPU9250_FIFO_COUNTH, &fifoH) != INV_SUCCESS)
		return 0;
	if (mpu_read_reg(MPU9250_FIFO_COUNTL, &fifoL) != INV_SUCCESS)
		return 0;

	return (fifoH << 8 ) | fifoL;
}

inv_error_t imu_updateFifo(void)
{
	short gyro[3], accel[3];
	unsigned long timestamp;
	unsigned char sensors, more;

	if (mpu_read_fifo(gyro, accel, &timestamp, &sensors, &more) != INV_SUCCESS)
		return INV_ERROR;

	if (sensors & INV_XYZ_ACCEL)
	{
		ax = accel[X_AXIS];
		ay = accel[Y_AXIS];
		az = accel[Z_AXIS];
	}
	if (sensors & INV_X_GYRO)
		gx = gyro[X_AXIS];
	if (sensors & INV_Y_GYRO)
		gy = gyro[Y_AXIS];
	if (sensors & INV_Z_GYRO)
		gz = gyro[Z_AXIS];

	time = timestamp;

	return INV_SUCCESS;
}

inv_error_t imu_setSensors(unsigned char sensors)
{
	return mpu_set_sensors(sensors);
}

bool imu_dataReady()
{
	unsigned char intStatusReg;

	if (mpu_read_reg(MPU9250_INT_STATUS, &intStatusReg) == INV_SUCCESS)
	{
		return (intStatusReg & (1<<INT_STATUS_RAW_DATA_RDY_INT));
	}
	return false;
}

inv_error_t imu_update(unsigned char sensors)
{
	inv_error_t aErr = INV_SUCCESS;
	inv_error_t gErr = INV_SUCCESS;
	inv_error_t mErr = INV_SUCCESS;
	inv_error_t tErr = INV_SUCCESS;

	if (sensors & UPDATE_ACCEL)
		aErr = imu_updateAccel();
	if (sensors & UPDATE_GYRO)
		gErr = imu_updateGyro();
	if (sensors & UPDATE_COMPASS)
		mErr = imu_updateCompass();
	if (sensors & UPDATE_TEMP)
		tErr = imu_updateTemperature();

	return aErr | gErr | mErr | tErr;
}

int imu_updateAccel(void)
{
	short data[3];

	if (mpu_get_accel_reg(data, &time))
	{
		return INV_ERROR;
	}
	ax = data[X_AXIS];
	ay = data[Y_AXIS];
	az = data[Z_AXIS];
	return INV_SUCCESS;
}

int imu_updateGyro(void)
{
	short data[3];

	if (mpu_get_gyro_reg(data, &time))
	{
		return INV_ERROR;
	}
	gx = data[X_AXIS];
	gy = data[Y_AXIS];
	gz = data[Z_AXIS];
	return INV_SUCCESS;
}

int imu_updateCompass(void)
{
	short data[3];

	if (mpu_get_compass_reg(data, &time))
	{
		return INV_ERROR;
	}
	mx = data[X_AXIS];
	my = data[Y_AXIS];
	mz = data[Z_AXIS];
	return INV_SUCCESS;
}

inv_error_t imu_updateTemperature(void)
{
	return mpu_get_temperature(&temperature, &time);
}

int imu_selfTest(unsigned char debug)
{

	int result = mpu_run_6500_self_test(gyro, accel,0);

	mpu_set_gyro_bias_reg(gyro);
	mpu_set_accel_bias_6500_reg(accel);

	return result;
}

inv_error_t imu_dmpBegin(unsigned short features, unsigned short fifoRate)
{
	unsigned short feat = features;
	unsigned short rate = fifoRate;

	if (imu_dmpLoad() != INV_SUCCESS)
		return INV_ERROR;

	// 3-axis and 6-axis LP quat are mutually exclusive.
	// If both are selected, default to 3-axis
	if (feat & DMP_FEATURE_LP_QUAT)
	{
		feat &= ~(DMP_FEATURE_6X_LP_QUAT);
		dmp_enable_lp_quat(1);
	}
	else if (feat & DMP_FEATURE_6X_LP_QUAT)
		dmp_enable_6x_lp_quat(1);

	if (feat & DMP_FEATURE_GYRO_CAL)
		dmp_enable_gyro_cal(1);

	if (imu_dmpEnableFeatures(feat) != INV_SUCCESS)
		return INV_ERROR;

	rate = constrain(rate, 1, 200);
	if (imu_dmpSetFifoRate(rate) != INV_SUCCESS)
		return INV_ERROR;

	return mpu_set_dmp_state(1);
}

inv_error_t imu_dmpLoad(void)
{
	return dmp_load_motion_driver_firmware();
}

unsigned short imu_dmpGetFifoRate(void)
{
	unsigned short rate;
	if (dmp_get_fifo_rate(&rate) == INV_SUCCESS)
		return rate;

	return 0;
}

inv_error_t imu_dmpSetFifoRate(unsigned short rate)
{
	if (rate > MAX_DMP_SAMPLE_RATE) rate = MAX_DMP_SAMPLE_RATE;
	return dmp_set_fifo_rate(rate);
}

inv_error_t imu_dmpUpdateFifo(void)
{
	short gyro[3];
	short accel[3];
	long quat[4];
	unsigned long timestamp;
	short sensors;
	unsigned char more;

	if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more)
		   != INV_SUCCESS)
    {
	   return INV_ERROR;
    }

	if (sensors & INV_XYZ_ACCEL)
	{
		ax = accel[X_AXIS];
		ay = accel[Y_AXIS];
		az = accel[Z_AXIS];
	}
	if (sensors & INV_X_GYRO)
		gx = gyro[X_AXIS];
	if (sensors & INV_Y_GYRO)
		gy = gyro[Y_AXIS];
	if (sensors & INV_Z_GYRO)
		gz = gyro[Z_AXIS];
	if (sensors & INV_WXYZ_QUAT)
	{
		qw = quat[0];
		qx = quat[1];
		qy = quat[2];
		qz = quat[3];
	}

	time = timestamp;

	return INV_SUCCESS;
}

inv_error_t imu_dmpEnableFeatures(unsigned short mask)
{
	unsigned short enMask = 0;
	enMask |= mask;
	// Combat known issue where fifo sample rate is incorrect
	// unless tap is enabled in the DMP.
	enMask |= DMP_FEATURE_TAP;
	return dmp_enable_feature(enMask);
}

unsigned short imu_dmpGetEnabledFeatures(void)
{
	unsigned short mask;
	if (dmp_get_enabled_features(&mask) == INV_SUCCESS)
		return mask;
	return 0;
}

inv_error_t imu_dmpSetTap(
        unsigned short xThresh, unsigned short yThresh, unsigned short zThresh,
        unsigned char taps, unsigned short tapTime, unsigned short tapMulti)
{
	unsigned char axes = 0;
	if (xThresh > 0)
	{
		axes |= TAP_X;
		xThresh = constrain(xThresh, 1, 1600);
		if (dmp_set_tap_thresh(1<<X_AXIS, xThresh) != INV_SUCCESS)
			return INV_ERROR;
	}
	if (yThresh > 0)
	{
		axes |= TAP_Y;
		yThresh = constrain(yThresh, 1, 1600);
		if (dmp_set_tap_thresh(1<<Y_AXIS, yThresh) != INV_SUCCESS)
			return INV_ERROR;
	}
	if (zThresh > 0)
	{
		axes |= TAP_Z;
		zThresh = constrain(zThresh, 1, 1600);
		if (dmp_set_tap_thresh(1<<Z_AXIS, zThresh) != INV_SUCCESS)
			return INV_ERROR;
	}
	if (dmp_set_tap_axes(axes) != INV_SUCCESS)
		return INV_ERROR;
	if (dmp_set_tap_count(taps) != INV_SUCCESS)
		return INV_ERROR;
	if (dmp_set_tap_time(tapTime) != INV_SUCCESS)
		return INV_ERROR;
	if (dmp_set_tap_time_multi(tapMulti) != INV_SUCCESS)
		return INV_ERROR;

    dmp_register_tap_cb(tap_cb);

	return INV_SUCCESS;
}

unsigned char imu_getTapDir(void)
{
	_tap_available = false;
	return tap_direction;
}

unsigned char imu_getTapCount(void)
{
	_tap_available = false;
	return tap_count;
}

bool imu_tapAvailable(void)
{
	return _tap_available;
}

inv_error_t imu_dmpSetOrientation(const signed char * orientationMatrix)
{
	unsigned short scalar;
	scalar = orientation_row_2_scale(orientationMatrix);
	scalar |= orientation_row_2_scale(orientationMatrix + 3) << 3;
	scalar |= orientation_row_2_scale(orientationMatrix + 6) << 6;

    dmp_register_android_orient_cb(orient_cb);

	return dmp_set_orientation(scalar);
}

unsigned char imu_dmpGetOrientation(void)
{
	return mpu9250_orientation;
}

inv_error_t imu_dmpEnable3Quat(void)
{
	unsigned short dmpFeatures;

	// 3-axis and 6-axis quat are mutually exclusive
	dmpFeatures = dmpGetEnabledFeatures();
	dmpFeatures &= ~(DMP_FEATURE_6X_LP_QUAT);
	dmpFeatures |= DMP_FEATURE_LP_QUAT;

	if (dmpEnableFeatures(dmpFeatures) != INV_SUCCESS)
		return INV_ERROR;

	return dmp_enable_lp_quat(1);
}

unsigned long imu_dmpGetPedometerSteps(void)
{
	unsigned long steps;
	if (dmp_get_pedometer_step_count(&steps) == INV_SUCCESS)
	{
		return steps;
	}
	return 0;
}

inv_error_t imu_dmpSetPedometerSteps(unsigned long steps)
{
	return dmp_set_pedometer_step_count(steps);
}

unsigned long imu_dmpGetPedometerTime(void)
{
	unsigned long walkTime;
	if (dmp_get_pedometer_walk_time(&walkTime) == INV_SUCCESS)
	{
		return walkTime;
	}
	return 0;
}

inv_error_t imu_dmpSetPedometerTime(unsigned long time)
{
	return dmp_set_pedometer_walk_time(time);
}

float imu_calcAccel(int axis)
{
	return (float) axis / (float) _aSense;
}

float imu_calcGyro(int axis)
{
	return (float) axis / (float) _gSense;
}

float imu_calcMag(int axis)
{
	return (float) axis / (float) _mSense;
}

float imu_calcQuat(long axis)
{
	return qToFloat(axis, 30);
}

float imu_qToFloat(long number, unsigned char q)
{
	unsigned long mask = 0;
	for (int i=0; i<q; i++)
	{
		mask |= (1<<i);
	}
	return (number >> q) + ((number & mask) / (float) (2<<(q-1)));
}

void imu_computeEulerAngles(bool degrees)
{
    float dqw = imu_qToFloat(qw, 30);
    float dqx = imu_qToFloat(qx, 30);
    float dqy = imu_qToFloat(qy, 30);
    float dqz = imu_qToFloat(qz, 30);

    float ysqr = dqy * dqy;
    float t0 = -2.0f * (ysqr + dqz * dqz) + 1.0f;
    float t1 = +2.0f * (dqx * dqy - dqw * dqz);
    float t2 = -2.0f * (dqx * dqz + dqw * dqy);
    float t3 = +2.0f * (dqy * dqz - dqw * dqx);
    float t4 = -2.0f * (dqx * dqx + ysqr) + 1.0f;

	// Keep t2 within range of asin (-1, 1)
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;

    pitch = asin(t2) * 2;
    roll = atan2(t3, t4);
    yaw = atan2(t1, t0);

	if (degrees)
	{
		pitch *= (180.0 / PI);
		roll *= (180.0 / PI);
		yaw *= (180.0 / PI);

		//Commented out below as we want angles in ± around 0
//		if (pitch < 0) pitch = 360.0 + pitch;
//		if (roll < 0) roll = 360.0 + roll;
//		if (yaw < 0) yaw = 360.0 + yaw;
	}
}

float imu_computeCompassHeading(void)
{
	if (my == 0)
		heading = (mx < 0) ? PI : 0;
	else
		heading = atan2(mx, my);

	if (heading > PI) heading -= (2 * PI);
	else if (heading < -PI) heading += (2 * PI);
	else if (heading < 0) heading += 2 * PI;

	heading*= 180.0 / PI;

	return heading;
}

unsigned short imu_orientation_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;		// error
    return b;
}

static void imu_tap_cb(unsigned char direction, unsigned char count)
{
	_tap_available = true;
	tap_count = count;
	tap_direction = direction;
}

static void imu_orient_cb(unsigned char orient)
{
	mpu9250_orientation = orient;
}


float constrain(float x, float a, float b){


	if(x >= a && x <= b){
		return x;
	}

	if(x < a){
		return a;
	}

	if(x > b){
		return b;
	}

}
