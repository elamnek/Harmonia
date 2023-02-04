// 
// 
// 

#include "main_motor.h"
#include <Servo.h>

Servo m_servoMainMotor;
int m_intMotorPinPWM = 6;
int m_intThrottle = 0;

void init_main_motor() {
	
	m_servoMainMotor.attach(m_intMotorPinPWM);
	delay(1);
	m_servoMainMotor.write(90);
	delay(5000);//need this delay to allow ESC to register neutral value (90=center of RC stick)
}


void commmand_main_motor(int intValue) {
	m_servoMainMotor.write(intValue);
	
	//convert to a more readible throttle value
	if (intValue < 90) {
		m_intThrottle = -90 + intValue;
	} else if (intValue > 90) {
		m_intThrottle = intValue-90;
	}
	else if (intValue == 90) {
		m_intThrottle = 0;
	}
}
	

int get_main_motor_throttle() {
	return m_intThrottle;
}


