// RTC.h

#ifndef _RTC_h
#define _RTC_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void init_rtc();

int getValue(String data, char separator, int index);

void set_rtc_time(String strDateTimeFromPC);

String get_rtc_time();

#endif

