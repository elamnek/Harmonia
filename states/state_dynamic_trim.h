// state_dynamic_trim.h

#ifndef _STATE_DYNAMIC_TRIM_h
#define _STATE_DYNAMIC_TRIM_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void init_dynamic_trim();

void adjust_dive_plane(double dblPitch);

#endif

