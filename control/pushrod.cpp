// 
// 
// 

#include "pushrod.h"

int m_intPushRodPinDir = 11;
int m_intPushRodPinPWM = 10;



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

int get_weight_pos() {
	return analogRead(A0);
}
