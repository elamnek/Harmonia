// servos.h

#ifndef _SERVOS_h
#define _SERVOS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void init_servos();

void CommandServo(String strCommand, int intValue);


#endif

