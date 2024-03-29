// state_run.h

#ifndef _STATE_RUN_h
#define _STATE_RUN_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void init_run(String strParams);

void run_start();

boolean adjust_run(float fltHeading, float fltPitch);


#endif

