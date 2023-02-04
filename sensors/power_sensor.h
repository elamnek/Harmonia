// power_sensor.h

#ifndef _POWER_SENSOR_h
#define _POWER_SENSOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

String init_powersensor();

float get_bus_voltage();

float get_shunt_voltage();

float get_current_mA();

float get_power_mW();


#endif

