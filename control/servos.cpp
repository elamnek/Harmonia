// 
// 
// 

#include "servos.h"
#include <Servo.h>

int m_intFwdDiveServoPin = 13;
int m_intAftDiveServoPin = 4;
int m_intAftRudderServoPin = 5;

Servo m_servoFwdDive;
Servo m_servoAftDive;
Servo m_servoAftRudder;

int m_intFwdDiveServoPos = 0;
int m_intAftDiveServoPos = 0;
int m_intAftRudderServoPos = 0;

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

int get_fwddiveplane_pos() {
	return m_intFwdDiveServoPos;
}
int get_aftdiveplane_pos() {
	return m_intAftDiveServoPos;
}
int get_aftrudder_pos() {
	return m_intAftRudderServoPos;
}


