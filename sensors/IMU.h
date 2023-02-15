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

float get_imuorientation_x(boolean blnChkRead);
float get_imuorientation_y(boolean blnChkRead);
float get_imuorientation_z(boolean blnChkRead);

void check_read();

#endif

