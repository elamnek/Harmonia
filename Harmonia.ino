/*
 Name:		Harmonia.ino
 Created:	9/11/2022 8:06:00 AM
 Author:	eugene lamnek
*/

//installed libraries
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
#include "control\pumps.h"
#include "control\main_motor.h"
#include "control\servos.h"
#include "control\pushrod.h"
#include "control\states.h"
#include "sensors\water_sensors.h"
#include "sensors\RTC.h"
#include "sensors\IMU.h"
#include "sensors\leonardo_sensors.h"
#include "sensors\pressure_sensor.h"
#include "comms\rf_comms.h"

#define DEPTH_TRIMMED (1<<0)
#define PITCH_TRIMMED (1<<1)


t_control_error depthError = {};
t_control_error trimError = {};

float dt = 1.0;
unsigned long tPrev= 0; 

int trimmed_state = 0;
float depthTargetDistance = 1.0;
float depthDistance = 0.0;
float pitchAngle = 0.0;
sensors_vec_t subOrientation = {};

auto timer1Hz = timer_create_default();


//FSM states
enum { IDLE, MANUAL, STATIC_TRIM, DYNAMIC_TRIM, RUN, ALARM} state;
//function used to return text description of current state
String  get_state() {
	switch (state) {
	case IDLE: return "IDLE";
	case MANUAL: return "MANUAL";
	case STATIC_TRIM: return "STATIC_TRIM";
	case DYNAMIC_TRIM: return "DYNAMIC_TRIM";
	case RUN: return "RUN";
	case ALARM: return "ALARM";
	}
}

float m_fltStaticTrimDepth;

void setup() {

	timer1Hz.every(1000, timer1Hz_interrupt);

	//always start in IDLE state
	state = IDLE;
  
	init_rtc();
	init_rf_comms();
	send_rf_comm("Harmonia is awake - time is: " + get_rtctime());

	init_servos();
	init_pumps();
	init_pushrod();
	init_main_motor();
	init_watersensors();
	init_leonardo_sensors();

	String msg = init_imu();
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
				
}

bool timer1Hz_interrupt(void*) {
	
	//every second all opartional data needs to be sent to remote (sensors, state, control commands etc.)
	send_operational_data_to_remote(get_state());

	return true;
}


void loop() {
	
	timer1Hz.tick();

	//update pitch angle and depth distance
	depth_distance = get_depth();
	subOrientation = get_imuorientation();
	pitchAngle = orientation.x;

	//call this on each loop - this updates sensor data coming from leonardo
	read_leonardo();

	//call this on each loop - this checks for new commands coming from desktop remote
	check_rf_comms();

	//set state using commands from remote
	String strRemoteCommand = get_remote_command();
	if (strRemoteCommand == "IDLE") { state = IDLE; }
	if (strRemoteCommand == "MANUAL") {state = MANUAL;}
	if (strRemoteCommand == "STATIC_TRIM") { 
		state = STATIC_TRIM;
		m_fltStaticTrimDepth = get_remote_param().toFloat();
	}
	if (strRemoteCommand == "DYNAMIC_TRIM") { state = DYNAMIC_TRIM; }
	if (strRemoteCommand == "RUN") { state = RUN; }
	if (strRemoteCommand == "ALARM") { state = ALARM; }
	
	//state control
	int intStartState = state;
	
	switch (state) {
	case IDLE:
		if (leak_detected()) {
			state = ALARM;
		}

		break;
	case MANUAL:

		//this checks for a manual command from RF remote and applies it
		apply_manual_command();

		break;
	case STATIC_TRIM:

		
		
		update_error((float)(depthTarget-depth_distance), dt, &depthError);
		update_error((float)(-pitchAngle) newErr, dt, &trimError);
		
		// Adjust the depth until the error is with in accpetable margin and has slowed to a near stop
		if ((abs(depthError.err/depthTarget)>0.05) || (abs(depthError.errDer)>0.01)) {
			adjust_depth(depthError);
			trimmed_state &= ~(DEPTH_TRIMMED);
		}
		else {
			trimmed_state |= DEPTH_TRIMMED;
		}
		
		// Adjust the pitch angle until error is with in accpetable margin and has slowed to a near stop
		if ((abs(pitchError.err)>0.1) || (abs(pitchError.errDer)>0.01)){
			adjust_trim(pitchError);
			trimmed_state &= ~(PITCH_TRIMMED);
		}
		else {
			trimmed_state |= PITCH_TRIMMED;
		}
		
		if (trimmed_state == (DEPTH_TRIMMED | PITCH_TRIMMED)) {
			command_pump("INFLATE", 0);
			state = DYNAMIC_TRIM;
		}
		


		//this is a non-blocking function that checks sensors and makes adjustments to trim
		//must be non-blocking so that main loop is always running and checking for leaks or new commands from remote
		//adjust_static_trim(m_fltStaticTrimDepth);


		break;
	case DYNAMIC_TRIM:
		break;
	case RUN:
		break;
	case ALARM:
		if (!leak_detected()) {
			state = IDLE;
		}
		
		break;
	}

	dt = (float)(millis()-tPrev);
	tPrev = millis();

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
