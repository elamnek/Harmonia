/*
 Name:		Harmonia.ino
 Created:	9/11/2022 8:06:00 AM
 Author:	eugene
*/

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include "sensors\IMU.h"
//#include "sensors\rpm_sensor.h"
#include <SPL06-007.h>
#include "sensors\air_pressure_sensor.h"
#include <MS5837.h>
#include "sensors\pressure_sensor.h"
#include <arduino-timer.h>
#include "control\pumps.h"
#include "sensors\water_sensors.h"
#include "control\servos.h"
#include "sensors\RTC.h"
#include <GravityRtc.h>
#include "Wire.h"
#include <Servo.h>

int m_intPushRodPinDir = 11;
int m_intPushRodPinPWM = 10;
int m_intMotorPinPWM = 6;
Servo m_servoMainMotor;


auto timer1Hz = timer_create_default();



String m_strRemoteCommand; //string to be captured from serial port
String m_strRemoteParam; //numeric parameter

String m_strLeonardo1;
String m_strLeonardo2;

//FSM states
enum { IDLE, STATIC_TRIM, DYNAMIC_TRIM, RUN, ALARM} state;
String  GetState() {
	switch (state) {
	case IDLE: return "IDLE";
	case STATIC_TRIM: return "STATIC_TRIM";
	case DYNAMIC_TRIM: return "DYNAMIC_TRIM";
	case RUN: return "RUN";
	case ALARM: return "ALARM";
	}
}


void setup() {

	timer1Hz.every(1000, timer1Hz_interrupt);

	state = IDLE;

	//data from HarmoniaRemote (RF)
	Serial1.begin(9600);

	Serial2.begin(9600);
	

	init_rtc();
	Serial1.println("Harmonia is awake - time is: " + get_rtctime());

	init_servos();
	init_pumps();
	init_watersensors();

	String msg = init_airpresssuresensor();
	if (msg.length() > 0) {
		Serial1.println(msg);
	}
	else {
		Serial1.println("air pressure sensor OK!!");
	}

	msg = init_imu();
	if (msg.length() > 0) {
		Serial1.println(msg);
	}
	else {
		Serial1.println("IMU sensor OK!!");
	}


	msg = init_presssuresensor(997);
	if (msg.length() > 0) {
		Serial1.println(msg);
	}
	else {
		Serial1.println("water pressure sensor OK!!");
	}


	//this reports on addresses of all connected I2C devices
	//had issues with connecting multiple pressure sensors
	//bluerobotics underwater sensor only support default I2C address
	//so does the internal pressure/temp sensor - so this is why it was moved
	//to the aft leonardo uC
	scan_i2c();
		
	m_servoMainMotor.attach(m_intMotorPinPWM);
	delay(1);
	m_servoMainMotor.write(90);
	delay(5000);//need this delay to allow ESC to register neutral value (90=center of RC stick)
	//ProgramESC();


	pinMode(m_intPushRodPinDir, OUTPUT);
	pinMode(m_intPushRodPinPWM, OUTPUT);
		
}

bool timer1Hz_interrupt(void*) {
	
	//Serial1.println(GetState() + "," + get_rtctime() + "," + String(leak_read()) + "," + String(get_altitude()) + "," + String(get_waterpressure()) + "," + 
	//	String(get_airpressure()) + "," + String(get_imuorientation().x) + "," + String(get_imuorientation().y) + "," + String(get_imuorientation().z) + "," + m_strLeonardo1);

		Serial1.println(m_strLeonardo1 + "," + m_strLeonardo2);

	return true;
}


void loop() {

	timer1Hz.tick();


	int intStartState = state;
	switch (state) {
	case IDLE:
		if (leak_detected()) {
			state = ALARM;
		}

		break;
	case STATIC_TRIM:
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
	
	
	//read data from leonardo (serial 2)
	boolean blnCommaHit = false;
	String strLeonardo1 = "";
	String strLeonardo2 = "";
	while (Serial2.available()) {
		delay(10);
		if (Serial2.available() > 0) {
			char c = Serial2.read();  //gets one byte from serial buffer
			if (c == ',') {
				blnCommaHit = true;
			}
			else {
				if (!blnCommaHit) {
					strLeonardo1 += c; //makes readstring from the single bytes
				}
				else {
					strLeonardo2 += c;
				}
			}
		}
	}
	if (strLeonardo1.length() > 0) {m_strLeonardo1 = strLeonardo1;}
	if (strLeonardo2.length() > 0) {m_strLeonardo2 = strLeonardo2; }


	//check for state change and send to remote - move this 2 the 1s timer event
	if (intStartState != state) {
		//note errors occur at remote end if we send a message via RF serial every iteration of the loop - so need to
		//only send when necessary
		Serial1.println("STATE=" + String(state));
	}

	boolean blnParamHit = false;
	while (Serial1.available()) {
		delay(10);
		if (Serial1.available() > 0) {
			char c = Serial1.read();  //gets one byte from serial buffer
			if (c == ',') { 
				blnParamHit = true; 
			}
			else {
				if (!blnParamHit) {
					m_strRemoteCommand += c; //makes readstring from the single bytes
				}
				else {
					m_strRemoteParam += c;
				}	
			}	
		}
	}

	if (m_strRemoteCommand.length() > 0) {
		Serial1.println("command: " + m_strRemoteCommand);  //so you can see the captured string 
		if (m_strRemoteParam.length() > 0) { 
			//m_strRemoteParam += '\0';
			Serial1.println("param: " + m_strRemoteParam);
			Serial1.println(m_strRemoteParam.toInt());
		}
											 									
		if (m_strRemoteCommand == "INFLATE") {
			Serial1.println("{inflating: now}");
			command_pump(m_strRemoteCommand, m_strRemoteParam.toInt());
		}
		else if (m_strRemoteCommand == "DEFLATE") {
			Serial1.println("deflating");
			command_pump(m_strRemoteCommand, m_strRemoteParam.toInt());
		}
		else if (m_strRemoteCommand == "FORWARD") {
			Serial1.println("forward");
			digitalWrite(m_intPushRodPinDir, HIGH);
			analogWrite(m_intPushRodPinPWM, m_strRemoteParam.toInt());
		}
		else if (m_strRemoteCommand == "REVERSE") {
			Serial1.println("reverse");
			digitalWrite(m_intPushRodPinDir, LOW);
			analogWrite(m_intPushRodPinPWM, m_strRemoteParam.toInt());
		}
		else if (m_strRemoteCommand == "PROPELL") {
			Serial1.println("propelling main motor");
			m_servoMainMotor.write(m_strRemoteParam.toInt());
		}
		else if (m_strRemoteCommand == "SERVOFWDDIVE") {
			Serial1.println("servo forward dive");		
			command_servo(m_strRemoteCommand, m_strRemoteParam.toInt());
		}
		else if (m_strRemoteCommand == "SERVOAFTDIVE") {
			Serial1.println("servo aft dive");
			command_servo(m_strRemoteCommand, m_strRemoteParam.toInt());
		}
		else if (m_strRemoteCommand == "SERVOAFTRUDDER") {
			Serial1.println("servo aft rudder");
			command_servo(m_strRemoteCommand, m_strRemoteParam.toInt());
		}

		m_strRemoteCommand = "";
		m_strRemoteParam = "";
	}
	
	/*Motor.write(120);
	delay(1000);
	Motor.write(110);
	delay(1000);
	Motor.write(100);
	delay(1000);
	Motor.write(90);
	delay(1000);
	Motor.write(80);
	delay(1000);
	Motor.write(70);
	delay(1000);
	Motor.write(60);
	delay(1000);*/
	
}

void scan_i2c() {

	byte error, address; //variable for error and I2C address
	int nDevices;

	Serial1.println("Scanning I2C...");

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
			Serial1.print("I2C device found at address 0x");
			if (address < 16)
				Serial1.print("0");
			Serial1.print(address, HEX);
			Serial1.println("  !");
			nDevices++;
		}
		else if (error == 4)
		{
			Serial1.print("Unknown error at address 0x");
			if (address < 16)
				Serial1.print("0");
			Serial1.println(address, HEX);
		}
	}
	if (nDevices == 0)
		Serial1.println("No I2C devices found\n");
	else
		Serial1.println("done\n");


}

void ProgramESC() {

	//Motor.write(0);//set key??
	//digitalWrite(m_intMotorPinPWM, LOW);
	
	m_servoMainMotor.write(100);//neutral
	delay(1000);
	m_servoMainMotor.write(200);//max within 2 seconds of ESC start
	delay(1500);
	m_servoMainMotor.write(0);//min
	delay(1500);
	m_servoMainMotor.write(100);//neutral
	delay(1500);
}

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
