// pushrod.h

#ifndef _PUSHROD_h
#define _PUSHROD_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void init_pushrod();

void command_pushrod(String strCommand, int intValue);

void command_pushrod_position(int intSetpoint);

int get_pushrod_pos();

void check_pushrod();


#endif

