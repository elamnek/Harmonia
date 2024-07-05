// depth_sensor.h

#ifndef _DEPTH_SENSOR_h
#define _DEPTH_SENSOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

String init_depthsensor(int intFluidDensity);

float get_depth();

float get_cached_depth();

float get_waterpressure();

float get_watertemperature();

//returns altitude in m above SL
float get_altitude();


#endif

