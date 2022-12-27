// this is the code that receives sensor data collected by the leonardo
// which is located in the aft pressure hull
// the leondardo collects RPM from the hall pressure sensor,
// and air pressure and temperature within the pressure hull using the
// SPL06-007 sensor

#include "leonardo_sensors.h"
#include <SPL06-007.h>

#define serialFromLeonardo Serial2

String m_strLeonardoRPM;
String m_strLeonardoPressure;
String m_strLeonardoTemp;

void init_leonardo_sensors() {

	serialFromLeonardo.begin(9600);

}

//this must be called each time main loop runs
void read_leonardo() {

	//read data from leonardo using serial2 port
	//format is RPM,pressure,temp
	int intValueIndex = 0; //this denotes the csv index of the value being read
	String strLeonardoRPM = "";
	String strLeonardoPressure = "";
	String strLeonardoTemp = "";
	while (serialFromLeonardo.available()) {
		delay(10);
		if (serialFromLeonardo.available() > 0) {
			char c = serialFromLeonardo.read();  //gets one byte from serial buffer
			if (c == ',') {
				intValueIndex++;
			}
			else {
				if (intValueIndex == 0) { strLeonardoRPM += c; }
				else if (intValueIndex == 1) { strLeonardoPressure += c; }
				else if (intValueIndex == 2) { strLeonardoTemp += c; }
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
