// GroveWaterSensor.h

#ifndef _GROVEWATERSENSOR_h
#define _GROVEWATERSENSOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void init_watersensors();

boolean leak_detected();

#endif

