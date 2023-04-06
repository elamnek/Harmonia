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

void command_servo_test(String strParams) {

	//note: blocking function that uses delays

	int intFwdDiveServo0Pos = get_sep_part(strParams, '|', 0);
	int intAftPitchServo0Pos = get_sep_part(strParams, '|', 1);
	int intAftRudderServo0Pos = get_sep_part(strParams, '|', 2);

	//set 0 positions of control planes
	command_servo("SERVOFWDDIVE", intFwdDiveServo0Pos, 0);
	command_servo("SERVOAFTDIVE", intAftPitchServo0Pos, 0);
	command_servo("SERVOAFTRUDDER", intAftRudderServo0Pos, 0);

	//hold on zero position for 2s
	delay(2000);

	//sweep servos
	servo_sweep("SERVOFWDDIVE", intFwdDiveServo0Pos, -40, 40, 25);
	servo_sweep("SERVOAFTDIVE", intAftPitchServo0Pos, -40, 40, 25);
	servo_sweep("SERVOAFTRUDDER", intAftRudderServo0Pos, -40, 40, 25);

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


