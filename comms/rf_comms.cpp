// 
// 
// 

#include "rf_comms.h"

//this serial will be used to communicate in 2 directions with the desktop software and digital twin
#define serialRF Serial1

String m_strRemoteCommand; //string to be captured from serial port
String m_strRemoteParam; //numeric parameter

void init_rf_comms() {
	
	//data from HarmoniaRemote (RF)
	serialRF.begin(9600);
}

void send_rf_comm(String strMsg) {
	serialRF.println(strMsg);
}

//this must be called each time main loop runs
void check_rf_comms() {

	int intValueIndex = 0; //this denotes the csv index of the value being read
	String strRemoteCommand = "";
	String strRemoteParam = "";
	while (serialRF.available()) {
		delay(10);
		if (serialRF.available() > 0) {
			char c = serialRF.read();  //gets one byte from serial buffer
			if (c == ',') {
				intValueIndex++;
			}
			else {
				if (intValueIndex == 0) { strRemoteCommand += c; }
				else if (intValueIndex == 1) { strRemoteParam += c; }
			}
		}
	}
	if (strRemoteCommand.length() > 0) { m_strRemoteCommand = strRemoteCommand; }
	if (strRemoteParam.length() > 0) { m_strRemoteParam = strRemoteParam; }
}
String get_remote_command() {
	return m_strRemoteCommand;
}
String get_remote_param() {
	return m_strRemoteParam;
}
void clear_rf_command() {
	m_strRemoteCommand = "";
	m_strRemoteParam = "";
}

