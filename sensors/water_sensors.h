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

boolean leak_detected2();

uint8_t leak_read();

#endif

