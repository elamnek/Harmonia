// bag_pressure_sensor.h

#ifndef _BAG_PRESSURE_SENSOR_h
#define _BAG_PRESSURE_SENSOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


String init_bagpressuresensor();

float get_bagpressure();


#endif

