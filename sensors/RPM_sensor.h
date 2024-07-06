// RPM_sensor.h

#ifndef _RPM_SENSOR_h
#define _RPM_SENSOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

String init_rpmsensor();

int32_t get_rpm(int intThrottle);

#endif

