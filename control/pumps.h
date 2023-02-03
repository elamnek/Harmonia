// pumps.h

#ifndef _PUMPS_h
#define _PUMPS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


void init_pumps();

void command_pump(String strCommand, int intValue);

int get_pump_status();

#endif

