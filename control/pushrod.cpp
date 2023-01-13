// 
// 
// 

#include "pushrod.h"

int m_intPushRodPinDir = 11;
int m_intPushRodPinPWM = 10;

int m_intMinPotValue = 305;
int m_intMaxPotValue = 988;


int m_Kp_pos = 1;
int m_minSpeed = 100;
int m_maxSpeed = 255;
int m_intSetpointPos = 0;


void init_pushrod() {

	pinMode(m_intPushRodPinDir, OUTPUT);
	pinMode(m_intPushRodPinPWM, OUTPUT);

}

void command_pushrod(String strCommand, int intValue) {

	if (strCommand == "FORWARD") {
		digitalWrite(m_intPushRodPinDir, HIGH);
		analogWrite(m_intPushRodPinPWM, intValue);
	}
	else if (strCommand == "REVERSE") {
		digitalWrite(m_intPushRodPinDir, LOW);
		analogWrite(m_intPushRodPinPWM, intValue);
	}

}

//moves the pushrod to a given percent value setpoint (100 = max forward position)
void command_pushrod_position(int intSetpoint) {
	m_intSetpointPos = intSetpoint;
}

//needs to be run each time main loop executes
void check_pushrod() {

	int intError = m_intSetpointPos - get_pushrod_pos();
	
	int intErrorMod = intError; //assume positive
	if (intError < 0) {intErrorMod = -intError;}

	int intSpeed = (intErrorMod * m_Kp_pos) + m_minSpeed;
	if (intSpeed > m_maxSpeed) { intSpeed = m_maxSpeed; } //saturate

	if (intError > 0) {
		command_pushrod("FORWARD", intSpeed);
	}
	else if (intError < 0) {
		command_pushrod("REVERSE", intSpeed);
	}
	else if (intError == 0) {
		command_pushrod("REVERSE", 0);
	}
	
}



int get_pushrod_pos() {
	float fltNum = analogRead(A0) - m_intMinPotValue;
	float fltDenom = m_intMaxPotValue - m_intMinPotValue;
	int intPercent = 100*fltNum/ fltDenom;
	return intPercent;
}
