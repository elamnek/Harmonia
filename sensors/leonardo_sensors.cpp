// this is the code that receives sensor data collected by the leonardo
// which is located in the aft pressure hull
// the leondardo collects RPM from the hall pressure sensor,
// and air pressure and temperature within the pressure hull using the
// SPL06-007 sensor
// it also collects air bag pressure from the MPRLS sensor
// and power data from the ina219 sensor

#include "leonardo_sensors.h"
#include <SPL06-007.h>

#define serialFromLeonardo Serial2

String m_strLeonardoRPM;
String m_strLeonardoPressure;
String m_strLeonardoTemp;
String m_strLeonardoBagPressure;
String m_strLeonardoBusVoltage;
String m_strLeonardoShuntVoltage;
String m_strLeonardoCurrent;
String m_strLeonardoPower;

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
	String strLeonardoBagPressure = "";
	String strLeonardoBusVoltage = "";
	String strLeonardoShuntVoltage = "";
	String strLeonardoCurrent = "";
	String strLeonardoPower = "";
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
				else if (intValueIndex == 3) { strLeonardoBagPressure += c; }
				else if (intValueIndex == 4) { strLeonardoBusVoltage += c; }
				else if (intValueIndex == 5) { strLeonardoShuntVoltage += c; }
				else if (intValueIndex == 6) { strLeonardoCurrent += c; }
				else if (intValueIndex == 7) { strLeonardoPower += c; }
			}
		}
	}
	if (strLeonardoRPM.length() > 0) { m_strLeonardoRPM = strLeonardoRPM; }
	if (strLeonardoPressure.length() > 0) { m_strLeonardoPressure = strLeonardoPressure; }
	if (strLeonardoTemp.length() > 0) { m_strLeonardoTemp = strLeonardoTemp; }
	if (strLeonardoBagPressure.length() > 0) { m_strLeonardoBagPressure = strLeonardoBagPressure; }
	if (strLeonardoBusVoltage.length() > 0) { m_strLeonardoBusVoltage = strLeonardoBusVoltage; }
	if (strLeonardoShuntVoltage.length() > 0) { m_strLeonardoShuntVoltage = strLeonardoShuntVoltage; }
	if (strLeonardoCurrent.length() > 0) { m_strLeonardoCurrent = strLeonardoCurrent; }
	if (strLeonardoPower.length() > 0) { m_strLeonardoPower = strLeonardoPower; }
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
float get_leonardo_bag_pressure() {
	return m_strLeonardoBagPressure.toFloat();
}
float get_leonardo_bus_voltage() {
	return m_strLeonardoBusVoltage.toFloat();
}
float get_leonardo_shunt_voltage() {
	return m_strLeonardoShuntVoltage.toFloat();
}
float get_leonardo_current() {
	return m_strLeonardoCurrent.toFloat();
}
float get_leonardo_power() {
	return m_strLeonardoPower.toFloat();
}
