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

void command_servo_test(String strParams);
void servo_sweep(String strCommand, int intServo0Value, int intMinPos, int intMaxPos, int intDelay_ms);

int get_fwddiveplane_pos();

int get_aftdiveplane_pos();

int get_aftrudder_pos();


#endif

