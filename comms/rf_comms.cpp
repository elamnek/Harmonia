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
	if (strLeonardoRPM.length() > 0) { m_strLeonardoRPM = strLeonardoRPM; }
	if (strLeonardoPressure.length() > 0) { m_strLeonardoPressure = strLeonardoPressure; }
	if (strLeonardoTemp.length() > 0) { m_strLeonardoTemp = strLeonardoTemp; }

}
float get_leonardo_rpm() {
	return m_strLeonardoRPM.toFloat();
}
float get_leonardo_pressure() {
	return m_strLeonardoPressure.toFloat();
}
float get_leonardo_temp() {
	return m_strLeonardoTemp.toFloat();
}
