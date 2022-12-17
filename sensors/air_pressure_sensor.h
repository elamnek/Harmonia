// air_pressure_sensor.h

#ifndef _AIR_PRESSURE_SENSOR_h
#define _AIR_PRESSURE_SENSOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

String init_airpresssuresensor();

double get_airpressure();


#endif

