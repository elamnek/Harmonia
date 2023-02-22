// state_static_trim.h

#ifndef _STATE_STATIC_TRIM_h
#define _STATE_STATIC_TRIM_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

//mode 0 is pure PID, mode 1 is modified pid
void init_static_trim(double dblDepthSetpoint, int intMode);

boolean adjust_depth();

void adjust_pitch(double dblPitch);


#endif

