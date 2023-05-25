// main_motor_precision.h

#ifndef _MAIN_MOTOR_PRECISION_h
#define _MAIN_MOTOR_PRECISION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void init_main_motor_precision();
void commmand_main_motor_precision(int intValue);
int get_main_motor_precision_throttle();

#endif

