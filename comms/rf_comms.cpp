// 
// 
// 

#include "rf_comms.h"
#include "..\sensors\water_sensors.h"
#include "..\sensors\RTC.h"
#include "..\sensors\IMU.h"
#include "..\sensors\leonardo_sensors.h"
#include "..\sensors\pressure_sensor.h"
#include "..\control\pushrod.h"

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

void send_operational_data_to_remote(String strState) {

	/// <summary>
	/// format of the data package is important
	/// each data value needs an associate metadataid (these are defined in the Digital Twin database in a table called dt_data_config
	/// each metadataid needs to have a matching record in the dt_data_config table
	/// the metadataid tells the DT where to insert the data (ie. what table/field combination)
	/// the first data value always has to be the rtc date/time (metadataid=13)
	/// format to use is {metadataid1:data_value1,metadataid2:data_value2,metadataid3:data_value3, etc.}
	/// use curly brackets either end to ensure that entire string is received at remote end and to distinguish from other messages going to remote
	/// </summary>
	send_rf_comm("{13:" + get_rtctime() + "," +
				  "3:" + strState + "," +
				  "2:" + String(leak_read()) + "," +
				  "1:" + String(get_waterpressure()) + ","
		          "7:" + String(get_leonardo_rpm()) + "," +
		          "10:" + String(get_leonardo_pressure()) + "," + 
		          "11:" + String(get_leonardo_temp()) + "," + 
		          "14:" + String(get_imuorientation().x) + "," + 
		          "15:" + String(get_imuorientation().y) + "," + 
		          "16:" + String(get_imuorientation().z) + "," + 
		          "17:" + String(get_pushrod_pos()) + "," +
		          "18:" + String(get_leonardo_bag_pressure()) +"}");


}

