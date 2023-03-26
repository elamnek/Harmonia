// helpers.h

#ifndef _HELPERS_h
#define _HELPERS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

int get_sep_part(String data, char separator, int index);
double get_sep_part_dbl(String data, char separator, int index);

#endif

