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

sensors_vec_t get_imuorientation();

sensors_event_t get_imu_event();

#endif

