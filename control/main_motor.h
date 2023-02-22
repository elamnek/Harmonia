// main_motor.h

#ifndef _MAIN_MOTOR_h
#define _MAIN_MOTOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void init_main_motor();

//main motor throttle commands should always use the scale -90 to +90 where 0 is motor off
void commmand_main_motor(int intValue);


int get_main_motor_throttle();

#endif

