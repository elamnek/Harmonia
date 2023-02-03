// 
// 
// 

#include "pumps.h"

int m_intPumpPinDir = 3;
int m_intPumpPinPWM = 2;
int m_intStatus = 0;

void init_pumps() {

	pinMode(m_intPumpPinDir, OUTPUT);
	pinMode(m_intPumpPinPWM, OUTPUT);

}

void command_pump(String strCommand, int intValue) {


	if (strCommand == "INFLATE") {
		digitalWrite(m_intPumpPinDir, HIGH);
		analogWrite(m_intPumpPinPWM, intValue);
		m_intStatus = intValue;
	}
	else if (strCommand == "DEFLATE") {
		digitalWrite(m_intPumpPinDir, LOW);
		analogWrite(m_intPumpPinPWM, intValue);
		m_intStatus = -intValue;
	}

}

int get_pump_status() {
	return m_intStatus;
}


