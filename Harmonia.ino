/*
 Name:		Harmonia.ino
 Created:	9/11/2022 8:06:00 AM
 Author:	eugene
*/



//#include <Adafruit_Sensor.h>
//#include <Adafruit_BNO055.h>


#include <GravityRtc.h>
#include "Wire.h"
#include <Servo.h>

//note rtc on mega need to use ports D20 and D21 by default - so connect the two SDA and SCL wires to these ports
GravityRtc rtc;     //RTC Initialization

int m_intPushRodPinDir = 11;
int m_intPushRodPinPWM = 10;
int m_intPumpPinDir = 3;
int m_intPumpPinPWM = 2;
int m_intMotorPinPWM = 6;

int m_intFwdDiveServoPin = 13;
int m_intAftDiveServoPin = 5;
int m_intAftRudderServoPin = 4;

Servo m_servoFwdDive;
Servo m_servoAftDive;
Servo m_servoAftRudder;
Servo m_servoMainMotor;

String m_strRemoteCommand; //string to be captured from serial port
String m_strRemoteParam; //numeric parameter

int m_intFwdWaterSensorPin = 31;
int m_intAftWaterSensorPin = 33;

void setup() {

	
	m_servoFwdDive.attach(m_intFwdDiveServoPin);
	m_servoAftDive.attach(m_intAftDiveServoPin);
	m_servoAftRudder.attach(m_intAftRudderServoPin);
	
	//m_servoFwdDive.write(90);
	/*m_servoAftDive.write(90);
	m_servoAftRudder.write(90);*/

	m_servoMainMotor.attach(m_intMotorPinPWM);
	delay(1);
	m_servoMainMotor.write(90);
	delay(5000);//need this delay to allow ESC to register neutral value (90=center of RC stick)
	//ProgramESC();

	Serial1.begin(9600);
	// put your setup code here, to run once:
	//pinMode(motor1pin1, OUTPUT);
	//pinMode(motor1pin2, OUTPUT);
	//pinMode(motor2pin1, OUTPUT);
	//pinMode(motor2pin2, OUTPUT);

	pinMode(m_intPushRodPinDir, OUTPUT);
	pinMode(m_intPushRodPinPWM, OUTPUT);
	pinMode(m_intPumpPinDir, OUTPUT);
	pinMode(m_intPumpPinPWM, OUTPUT);
	
	pinMode(m_intFwdWaterSensorPin, INPUT);
	pinMode(m_intAftWaterSensorPin, INPUT);


	//rtc.setup();
	//Set the RTC time automatically: Calibrate RTC time by your computer time
	//rtc.adjustRtc(F(__DATE__), F(__TIME__));
	//rtc.adjustRtc(2017, 6, 19, 1, 12, 7, 0);  //Set time: 2017/6/19, Monday, 12:07:00

}


void loop() {

	if (digitalRead(m_intFwdWaterSensorPin) == 0 || digitalRead(m_intAftWaterSensorPin) == 0) {
		Serial1.println("ALARM!!!!!!");
	}
	
	//rtc.read();
	////*************************Time********************************
	//Serial.print("   Year = ");//year
	//Serial.print(rtc.year);
	//Serial.print("   Month = ");//month
	//Serial.print(rtc.month);
	//Serial.print("   Day = ");//day
	//Serial.print(rtc.day);
	//Serial.print("   Week = ");//week
	//Serial.print(rtc.week);
	//Serial.print("   Hour = ");//hour
	//Serial.print(rtc.hour);
	//Serial.print("   Minute = ");//minute
	//Serial.print(rtc.minute);
	//Serial.print("   Second = ");//second
	//Serial.println(rtc.second);
	
	
	//digitalWrite(m_intPushRodPinDir, HIGH);
	//analogWrite(m_intPushRodPinPWM, 100);

	//digitalWrite(m_intPumpPinDir, HIGH);
	//analogWrite(m_intPumpPinPWM, 0);

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
			digitalWrite(m_intPumpPinDir, HIGH);
			analogWrite(m_intPumpPinPWM, m_strRemoteParam.toInt());
		}
		else if (m_strRemoteCommand == "DEFLATE") {
			Serial1.println("deflating");
			digitalWrite(m_intPumpPinDir, LOW);
			analogWrite(m_intPumpPinPWM, m_strRemoteParam.toInt());
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
			Serial1.println("propell");
			m_servoMainMotor.write(m_strRemoteParam.toInt());
		}
		else if (m_strRemoteCommand == "SERVOFWDDIVE") {
			Serial1.println("servo forward dive");
			m_servoFwdDive.write(m_strRemoteParam.toInt());
		}
		else if (m_strRemoteCommand == "SERVOAFTDIVE") {
			Serial1.println("servo aft dive");
			m_servoAftDive.write(m_strRemoteParam.toInt());
		}
		else if (m_strRemoteCommand == "SERVOAFTRUDDER") {
			Serial1.println("servo aft rudder");
			m_servoAftRudder.write(m_strRemoteParam.toInt());
		}

		m_strRemoteCommand = "";
		m_strRemoteParam = "";
	}
	/*else {
		Serial.println("no RC");
	}*/


	



	
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
