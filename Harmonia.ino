/*
 Name:		Harmonia.ino
 Created:	9/11/2022 8:06:00 AM
 Author:	eugene lamnek
*/

//installed libraries



#include <DFRobot_INA219.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPL06-007.h>
#include <MS5837.h>
#include <arduino-timer.h>
#include <GravityRtc.h>
#include "Wire.h"
#include <Servo.h>

//harmonia libraries
#include "states\state_manual.h"
#include "states\state_static_trim.h"
#include "states\state_dynamic_trim.h"
#include "states\state_run.h"
#include "control\pumps.h"
#include "control\main_motor.h"
#include "control\servos.h"
#include "control\pushrod.h"
#include "sensors\water_sensors.h"
#include "sensors\RTC.h"
#include "sensors\IMU.h"
#include "sensors\leonardo_sensors.h"
#include "sensors\pressure_sensor.h"
#include "sensors\power_sensor.h"
#include "comms\rf_comms.h"
#include "data\sdcard.h"

int m_intCounter = 0;

auto timer2Hz = timer_create_default();
auto timer5Hz = timer_create_default();

//FSM states
enum { IDLE, MANUAL, STATIC_TRIM, DYNAMIC_TRIM, RUN, ALARM, UPLOAD} state;
//function used to return text description of current state
String  get_state() {
	switch (state) {
	case IDLE: return "IDLE";
	case MANUAL: return "MANUAL";
	case STATIC_TRIM: return "STATIC_TRIM";
	case DYNAMIC_TRIM: return "DYNAMIC_TRIM";
	case RUN: return "RUN";
	case ALARM: return "ALARM";
	case UPLOAD: return "UPLOAD";
	}
}

void setup() {

	//always start in IDLE state
	state = IDLE;
  
	init_rtc();
	init_rf_comms();
	send_rf_comm("Harmonia is awake - stored time is: " + get_rtc_time());

	init_servos();
	init_pumps();
	init_pushrod();
	init_main_motor();
	init_watersensors();
	init_leonardo_sensors();

	String msg = init_sdcard();
	if (msg.length() > 0) {
		send_rf_comm(msg);
	}
	else {
		send_rf_comm("SDCard OK!!");
	}

	msg = init_imu();
	if (msg.length() > 0) {
		send_rf_comm(msg);
	}
	else {
		send_rf_comm("IMU sensor OK!!");
	}
	
	msg = init_presssuresensor(997);
	if (msg.length() > 0) {
		send_rf_comm(msg);
	}
	else {
		send_rf_comm("water pressure sensor OK!!");
	}

	//this scan reports on addresses of all connected I2C devices
	//I had issues with connecting multiple pressure sensors
	//bluerobotics underwater sensor only support default I2C address
	//so does the internal pressure/temp sensor, which resulted in 2 sensors with same address 
	//- so this is why internal pressure sensor was moved to the aft leonardo uC
	scan_i2c();

	//start interupts
	timer2Hz.every(500, timer2Hz_interrupt);
	timer5Hz.every(200, timer5Hz_interrupt);
				
}

bool timer2Hz_interrupt(void*) {

	
	/// format of the data package is important
	/// each data value needs an associate metadataid (these are defined in the Digital Twin database in a table called dt_data_config
	/// each metadataid needs to have a matching record in the dt_data_config table
	/// the metadataid tells the DT where to insert the data (ie. what table/field combination)
	/// the first data value always has to be the rtc date/time (metadataid=13)
	/// format to use is {metadataid1|data_value1,metadataid2|data_value2,metadataid3|data_value3, etc.}
	/// use curly brackets either end to ensure that entire string is received at remote end and to distinguish from other messages going to remote	
	
	////send_rf_comm(String(m_fltOrientation_x) + "," + String(m_fltOrientation_y) + "," + String(m_fltOrientation_z));

	//unsigned long lngStart = millis();

	read_imu();

	String strData = "{13|" + get_rtc_time() + "," +
						"4|" + get_state() + "," +
						"2|" + String(fwd_leak_detected()) + "," +
						"3|" + String(aft_leak_detected()) + "," +
						"1|" + String(get_depth()) + ","
						"7|" + get_leonardo_rpm_str() + "," +
						"10|" + get_leonardo_pressure_str() + "," +
						"11|" + get_leonardo_temp_str() + "," +
						"14|" + String(get_imuorientation_x()) + "," + //heading
						"15|" + String(get_imuorientation_y()) + "," + //pitch
						"16|" + String(get_imuorientation_z()) + "," + //roll
						"17|" + String(get_pushrod_pos()) + "," +
						"19|" + String(get_waterpressure()) + "," +
						"18|" + get_leonardo_bag_pressure_str() + "," +
						"21|" + String(get_pump_status()) + "," +
						"22|" + String(get_main_motor_throttle()) + "," +
		                "23|" + get_leonardo_bus_voltage_str() + "," +
		                "24|" + get_leonardo_shunt_voltage_str() + "," +
		                "25|" + get_leonardo_current_str() + "," +
		                "26|" + get_leonardo_power_str() +
						"}";

	/*unsigned long lngElapsed = millis() - lngStart;
	strData = strData + String(lngElapsed);*/

	m_intCounter = m_intCounter + 1;
	if (m_intCounter < 2) {
		return true;
	}
	else {
		m_intCounter = 0;
	}

	//every second all operational data needs to be sent to remote (sensors, state, control commands etc.)
	send_rf_comm(strData);

	//if in the upload state - we don't want data to be stored
	if (state == UPLOAD) { return true; }

	//save to sdcard
	sdcard_save_data(strData);

	return true;
}

bool timer5Hz_interrupt(void*) {

	//this function stored data that needs to be captured at higher fidelity
	//5Hz data doesn't need to be sent to remote display, only stored to SD card (except if in upload mode)

	


	//if in the upload state - we don't want data to be stored
	if (state == UPLOAD) { return true; }

	String strData = "{13|" + get_rtc_time() + "," +
		              "27|" + get_fwddiveplane_angle() + "," +
		              "28|" + get_aftdiveplane_angle() + "," +
		              "29|" + get_aftrudder_angle() + 
		              "}";

	//save to sdcard
	sdcard_save_data(strData);

	return true;
}


void loop() {
	
	timer2Hz.tick();
	timer5Hz.tick();

	//check leak sensors and override any state that has been set
	if (fwd_leak_detected() == 1 || aft_leak_detected() == 1) { 
		state = ALARM; 
	}
	else {
		//ignore these if leak detected
		read_leonardo(); //this updates sensor data coming from leonardo	
		//check_pushrod(); //adjusts position of pushrod based on latest setpoint command
	}

	//check for new commands coming from desktop remote
	check_rf_comms(); 

	//set state using commands from remote
	String strRemoteCommand = get_remote_command();
	if (state == ALARM) {
		//system in alarm state - can only be changed by command to go to idle state
		if (strRemoteCommand == "IDLE") { state = IDLE; }
	}
	else {
		//system NOT in alarm state - normal state changes allowed
		if (strRemoteCommand == "IDLE") { state = IDLE; }
		if (strRemoteCommand == "MANUAL") { state = MANUAL; }
		if (strRemoteCommand == "STATIC_TRIM") {
			state = STATIC_TRIM;
			init_static_trim(get_remote_param().toDouble(),1);
			clear_rf_command();
		}
		if (strRemoteCommand == "DYNAMIC_TRIM") { 
			state = DYNAMIC_TRIM; 
			init_dynamic_trim();
			clear_rf_command();
		}

		if (strRemoteCommand == "RUN") { 
			state = RUN; 
			init_run();
			clear_rf_command();
		}

		if (strRemoteCommand == "ALARM") { state = ALARM; }
		if (strRemoteCommand == "UPLOAD") { state = UPLOAD; }
	}

	//check for command to set time
	if (strRemoteCommand == "TIMESET") {
		set_rtc_time(get_remote_param());
		clear_rf_command();
	}

	//state control	
	switch (state) {
	case IDLE:

		//in idle state need to stop any active operation
		command_pump("INFLATE", 0);
		commmand_main_motor(0);

		break;
	case MANUAL:

		//this checks for a manual command from RF remote and applies it
		apply_manual_command();

		break;
	case STATIC_TRIM:
	
		if (adjust_depth()) {
			//only adjust pitch if depth is within tolerance
			adjust_pitch(get_imuorientation_y());
		}
		
		break;
	case DYNAMIC_TRIM:

		//float test = get_imuorientation_y(false);
		//double test2 = test;

		adjust_dive_plane(get_imuorientation_y());


		break;
	case RUN:

		adjust_run();

		break;
	case ALARM:
		//once in alarm state can only exit by user changing to another state via remote software
		
		//inflating will help sub return to surface and also ensure it stops pumping in water if the air bag is ruptured
		command_pump("INFLATE", 255);

		//could also initiate power to surface using motor at high thrust and rudders but this would be a problem in confined space such as tank
		//collision with walls could occur

		//also clear the comms buffer so that any user command to idle state can be read
		clear_rf_command();

		break;
	case UPLOAD:
		//in upload state need to stop any active operation
		command_pump("INFLATE", 0);
		commmand_main_motor(0);


		break;
	}
	
	
}

void scan_i2c() {

	byte error, address; //variable for error and I2C address
	int nDevices;

	send_rf_comm("Scanning I2C...");

	nDevices = 0;
	for (address = 1; address < 127; address++)
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0)
		{
			nDevices++;
			String strMsg = "I2C device found at address 0x";
			if (address < 16){ strMsg = strMsg + "0";}
			strMsg = strMsg + String(address, HEX);
			send_rf_comm(strMsg);
			
		}
		else if (error == 4)
		{
			String strMsg = "Unknown error at address 0x";
			if (address < 16) { strMsg = strMsg + "0"; }
			strMsg = strMsg + String(address, HEX);
			send_rf_comm(strMsg);
		}
	}
	if (nDevices == 0)
		send_rf_comm("No I2C devices found");
	else
		send_rf_comm("done");


}


//dead car bodies

//update pitch angle and depth distance
	/*depth_distance = get_depth();
	subOrientation = get_imuorientation();
	pitchAngle = subOrientation.x;*/

//update_error((float)(depthTargetDistance-depth_distance), dt, &depthError);
		//update_error((float)(-pitchAngle), dt, &trimError);
		//
		//// Adjust the depth until the error is with in accpetable margin and has slowed to a near stop
		//if ((abs(depthError.err/ depthTargetDistance)>0.05) || (abs(depthError.errDer)>0.01)) {
		//	adjust_depth(depthError);
		//	trimmed_state &= ~(DEPTH_TRIMMED);
		//}
		//else {
		//	trimmed_state |= DEPTH_TRIMMED;
		//}
		//
		//// Adjust the pitch angle until error is with in accpetable margin and has slowed to a near stop
		//if ((abs(pitchError.err)>0.1) || (abs(pitchError.errDer)>0.01)){
		//	adjust_static_trim(pitchError);
		//	trimmed_state &= ~(PITCH_TRIMMED);
		//}
		//else {
		//	trimmed_state |= PITCH_TRIMMED;
		//}
		//
		//if (trimmed_state == (DEPTH_TRIMMED | PITCH_TRIMMED)) {
		//	command_pump("INFLATE", 0);
		//	state = DYNAMIC_TRIM;
		//}



		//this is a non-blocking function that checks sensors and makes adjustments to trim
		//must be non-blocking so that main loop is always running and checking for leaks or new commands from remote
		//adjust_static_trim(m_fltStaticTrimDepth);

//check for state change and send to remote - move this 2 the 1s timer event
	//if (intStartState != state) {
	//	//note errors occur at remote end if we send a message via RF serial every iteration of the loop - so need to
	//	//only send when necessary
	//	send_rf_comm("STATE=" + String(state));
	//}

//void ProgramESC() {
//
//	//Motor.write(0);//set key??
//	//digitalWrite(m_intMotorPinPWM, LOW);
//	
//	m_servoMainMotor.write(100);//neutral
//	delay(1000);
//	m_servoMainMotor.write(200);//max within 2 seconds of ESC start
//	delay(1500);
//	m_servoMainMotor.write(0);//min
//	delay(1500);
//	m_servoMainMotor.write(100);//neutral
//	delay(1500);
//}

////Check to see if anything is available in the serial receive buffer
//while (Serial.available() > 0)
//{
//
//	//Create a place to hold the incoming message
//	static char message[MAX_MESSAGE_LENGTH];
//	static unsigned int message_pos = 0;
//
//	//Read the next available byte in the serial receive buffer
//	char inByte = Serial.read();
//
//	//Message coming in (check not terminating character) and guard for over message size
//	if (inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH))
//	{
//		//Add the incoming byte to our message
//		message[message_pos] = inByte;
//		message_pos++;
//	}
//	//Full message received...
//	else
//	{
//		//Add null character to string
//		message[message_pos] = '\0';
//
//		//Print the message (or do other things)
//		Serial.println(message);
//
//		String strCommand = String(message);
//		Serial.println("command: " + strCommand);
//
//
//		if (strCommand == "12345") {
//			Serial.println("{inflating: now}");
//			digitalWrite(m_intPumpPinDir, HIGH);
//			analogWrite(m_intPumpPinPWM, 0);
//		}
//		else if (strCommand == "DEF") {
//			Serial.println("deflating");
//			digitalWrite(m_intPumpPinDir, LOW);
//			analogWrite(m_intPumpPinPWM, 255);
//		}
//		else if (strCommand == "FOR") {
//			Serial.println("forward");
//			digitalWrite(m_intPushRodPinDir, HIGH);
//
//			/*if (m_strRemoteParam.length() > 0) {
//				m_strRemoteParam.trim();
//				analogWrite(m_intPushRodPinPWM, m_strRemoteParam.toInt());
//			}
//			else {
//				analogWrite(m_intPushRodPinPWM, 100);
//			}*/
//
//		}
//		else if (strCommand == "REV") {
//			Serial.println("reverse");
//			digitalWrite(m_intPushRodPinDir, LOW);
//			analogWrite(m_intPushRodPinPWM, 100);
//		}
//
//
//
//		//Reset for the next message
//		message_pos = 0;
//	}
//}
