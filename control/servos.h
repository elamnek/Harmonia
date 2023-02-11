// servos.h

#ifndef _SERVOS_h
#define _SERVOS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void init_servos();

void command_servo(String strCommand, int intValue);

float get_fwddiveplane_angle();

float get_aftdiveplane_angle();

float get_aftrudder_angle();


#endif

