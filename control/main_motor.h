// main_motor.h

#ifndef _MAIN_MOTOR_h
#define _MAIN_MOTOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void init_main_motor();

void commmand_main_motor(int intValue);

int get_main_motor_throttle();

#endif

