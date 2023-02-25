// sdcard.h

#ifndef _SDCARD_h
#define _SDCARD_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

String init_sdcard();

void sdcard_save_data(String strDataLine);

void sdcard_record_count();

void sdcard_upload_data();

#endif

