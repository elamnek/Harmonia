// state_run_2.h

#ifndef _STATE_RUN_2_h
#define _STATE_RUN_2_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void init_run_2(String strParams);

void run_start_2();

boolean adjust_run_2(float fltHeading, float fltPitch);

#endif

