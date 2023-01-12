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

int get_weight_pos();

#endif

