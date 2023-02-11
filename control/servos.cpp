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

void init_servos() {
	
	m_servoFwdDive.attach(m_intFwdDiveServoPin);
	m_servoAftDive.attach(m_intAftDiveServoPin);
	m_servoAftRudder.attach(m_intAftRudderServoPin);

	m_servoFwdDive.write(120);
	m_servoAftDive.write(120);
	m_servoAftRudder.write(120);
	
}

void command_servo(String strCommand,int intValue) {

	if (strCommand == "SERVOFWDDIVE") {
		m_servoFwdDive.write(intValue);
	}
	else if (strCommand == "SERVOAFTDIVE"){
		m_servoAftDive.write(intValue);
	}
	else if (strCommand == "SERVOAFTRUDDER"){
		m_servoAftRudder.write(intValue);
	}
	
}

float get_fwddiveplane_angle() {

}
float get_aftdiveplane_angle() {

}
float get_aftrudder_angle() {

}


