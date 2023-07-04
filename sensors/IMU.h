// IMU.h

#ifndef _IMU_h
#define _IMU_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

String init_imu();

String check_imu_calibration();

String get_imucal();

void read_imu();

float get_imuorientation_x();
float get_imuorientation_y();
float get_imuorientation_z();

float get_imuacceleration_x();
float get_imuacceleration_y();
float get_imuacceleration_z();

void check_read();

#endif

