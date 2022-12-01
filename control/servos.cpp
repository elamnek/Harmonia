// 
// 
// 

#include "servos.h"
#include <Servo.h>

int m_intFwdDiveServoPin = 13;
int m_intAftDiveServoPin = 5;
int m_intAftRudderServoPin = 4;

Servo m_servoFwdDive;
Servo m_servoAftDive;
Servo m_servoAftRudder;

void init_servos() {
	
	m_servoFwdDive.attach(m_intFwdDiveServoPin);
	m_servoAftDive.attach(m_intAftDiveServoPin);
	m_servoAftRudder.attach(m_intAftRudderServoPin);

	m_servoFwdDive.write(120);
	m_servoAftDive.write(120);
	m_servoAftRudder.write(120);
	//asdfahsfdkj
}

void CommandServo(String strCommand,int intValue) {

	if (strCommand == "SERVOFWDDIVE") {
		m_servoFwdDive.write(intValue);
	}
	
}


