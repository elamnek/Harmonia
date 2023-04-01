// servos.h

#ifndef _SERVOS_h
#define _SERVOS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void init_servos();

void command_servo(String strCommand, int intValue, int intPos);

int get_fwddiveplane_pos();

int get_aftdiveplane_pos();

int get_aftrudder_pos();


#endif

