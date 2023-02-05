// leonardo_sensors.h

#ifndef _LEONARDO_SENSORS_h
#define _LEONARDO_SENSORS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void init_leonardo_sensors();

void read_leonardo();

float get_leonardo_rpm();

float get_leonardo_pressure();

float get_leonardo_temp();

float get_leonardo_bag_pressure();

float get_leonardo_bus_voltage();

float get_leonardo_shunt_voltage();

float get_leonardo_current();

float get_leonardo_power();

#endif

