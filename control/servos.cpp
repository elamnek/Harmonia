// 
// 
// 

#include "servos.h"
#include <Servo.h>
#include "..\helpers.h"

int m_intFwdDiveServoPin = 13;
int m_intAftDiveServoPin = 4;
int m_intAftRudderServoPin = 4; //this was pin 5 but rudder is now using dive plane servo

Servo m_servoFwdDive;
Servo m_servoAftDive;
Servo m_servoAftRudder;

int m_intFwdDiveServoPos = 0;
int m_intAftDiveServoPos = 0;
int m_intAftRudderServoPos = 0;

int m_intCurrentRudder = 148;

void init_servos() {
	
	m_servoFwdDive.attach(m_intFwdDiveServoPin);
	m_servoAftDive.attach(m_intAftDiveServoPin);
	m_servoAftRudder.attach(m_intAftRudderServoPin);

	m_servoFwdDive.write(120);
	m_servoAftDive.write(120);
	m_servoAftRudder.write(120);
	
}

void command_servo(String strCommand,int intValue,int intPos) {

	if (strCommand == "SERVOFWDDIVE") {
		m_servoFwdDive.write(intValue);
		m_intFwdDiveServoPos = intPos;
	}
	else if (strCommand == "SERVOAFTDIVE"){
		m_servoAftDive.write(intValue);
		m_intAftDiveServoPos = intPos;
	}
	else if (strCommand == "SERVOAFTRUDDER"){
		m_servoAftRudder.write(intValue);
		m_intAftRudderServoPos = intPos;
	}	
}
void command_servo_relative(String strCommand, int intValue) {
	if (strCommand == "RUDDER") {
		m_intCurrentRudder = m_intCurrentRudder + intValue;
		if (m_intCurrentRudder > 180) { m_intCurrentRudder = 180; }
		if (m_intCurrentRudder < 0) { m_intCurrentRudder = 0; }

		m_servoAftRudder.write(m_intCurrentRudder);
		
	}
}

void command_servo_test(String strParams) {

	//note: blocking function that uses delays

	int intFwdDiveServo0Pos = get_sep_part(strParams, '|', 0);
	int intAftPitchServo0Pos = get_sep_part(strParams, '|', 1);
	int intAftRudderServo0Pos = get_sep_part(strParams, '|', 2);

	
	//sweep servos
	if (intFwdDiveServo0Pos != 555) {
		command_servo("SERVOFWDDIVE", intFwdDiveServo0Pos, 0);
		delay(2000);
		servo_sweep("SERVOFWDDIVE", intFwdDiveServo0Pos, -40, 40, 25);
		command_servo("SERVOFWDDIVE", intFwdDiveServo0Pos, 0);
	}
	if (intAftPitchServo0Pos != 555) {
		command_servo("SERVOAFTDIVE", intAftPitchServo0Pos, 0);
		delay(2000);
		servo_sweep("SERVOAFTDIVE", intAftPitchServo0Pos, -40, 40, 25);
		command_servo("SERVOAFTDIVE", intAftPitchServo0Pos, 0);
	}
	if (intAftRudderServo0Pos != 555) {
		command_servo("SERVOAFTRUDDER", intAftRudderServo0Pos, 0);
		delay(2000);
		servo_sweep("SERVOAFTRUDDER", intAftRudderServo0Pos, -40,40, 25);
		command_servo("SERVOAFTRUDDER", intAftRudderServo0Pos, 0);
	}

}
void servo_sweep(String strCommand,int intServo0Value,int intMinPos,int intMaxPos,int intDelay_ms){
	for (int pos = intMinPos; pos <= intMaxPos; pos++) {
		command_servo(strCommand, intServo0Value + pos, pos);
		delay(intDelay_ms);
	}
	for (int pos = intMaxPos; pos >= intMinPos; --pos) {
		command_servo(strCommand, intServo0Value + pos, pos);
		delay(intDelay_ms);
	}
}

int get_fwddiveplane_pos() {
	return m_intFwdDiveServoPos;
}
int get_aftdiveplane_pos() {
	return m_intAftDiveServoPos;
}
int get_aftrudder_pos() {
	return m_intAftRudderServoPos;
}


