// rf_comms.h

#ifndef _RF_COMMS_h
#define _RF_COMMS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void init_rf_comms();

void send_rf_comm(String strMsg);

void check_rf_comms();

String get_remote_command();

String get_remote_param();

void clear_rf_command();

#endif

