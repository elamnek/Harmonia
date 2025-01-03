// state_run_3.h

#ifndef _STATE_RUN_3_h
#define _STATE_RUN_3_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void init_run_3(String strParams);


void run_start_3(double dblX);

boolean adjust_run_3(double dblX,double dblVelocityX);

double get_maxdvlvelocity();
double get_dvlacceleration();
double get_maxdvlacceleration();
double get_dvlcrashstopdist();

#endif

