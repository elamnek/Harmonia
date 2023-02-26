// state_static_trim_2.h

#ifndef _STATE_STATIC_TRIM_2_h
#define _STATE_STATIC_TRIM_2_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void init_static_trim_2(float fltDepthSetpoint);

void adjust_depth_2();

float get_dive_rate_2();

void adjust_pitch_2(float fltPitch);

#endif

