// 
// 
// 

#include "main_motor.h"

Servo m_servoMainMotor;
int m_intMotorPinPWM = 6;

void init_main_motor() {
	
	m_servoMainMotor.attach(m_intMotorPinPWM);
	delay(1);
	m_servoMainMotor.write(90);
	delay(5000);//need this delay to allow ESC to register neutral value (90=center of RC stick)
}


void commmand_main_motor(int intValue) {
	m_servoMainMotor.write(intValue);
}


