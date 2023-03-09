// 
// 
// 

#include "pumps.h"
#include "..\comms\LEDs.h"

int m_intPumpPinDir = 3;
int m_intPumpPinPWM = 2;
int m_intStatus = 0;

void init_pumps() {

	pinMode(m_intPumpPinDir, OUTPUT);
	pinMode(m_intPumpPinPWM, OUTPUT);

}

void command_pump(String strCommand, int intValue) {

	
	//default to both LEDs off
	orange_led_off();
	yellow_led_off();

	if (strCommand == "INFLATE") {
		digitalWrite(m_intPumpPinDir, HIGH);
		analogWrite(m_intPumpPinPWM, intValue);
		m_intStatus = intValue;
		if (intValue > 0) {
			orange_led_on();
			yellow_led_off();
		}	
	}
	else if (strCommand == "DEFLATE") {
		digitalWrite(m_intPumpPinDir, LOW);
		analogWrite(m_intPumpPinPWM, intValue);
		m_intStatus = -intValue;
		if (intValue > 0) {
			yellow_led_on();
			orange_led_off();
		}	
	}
}

int get_pump_status() {
	return m_intStatus;
}


