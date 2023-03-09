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
	red_led_off(); //red for deflate
	yellow_led_off(); //yellow for inflate

	if (strCommand == "INFLATE") {
		digitalWrite(m_intPumpPinDir, HIGH);
		analogWrite(m_intPumpPinPWM, intValue);
		m_intStatus = intValue;
		if (intValue > 0) {
			yellow_led_on();
			red_led_off();
		}	
	}
	else if (strCommand == "DEFLATE") {
		digitalWrite(m_intPumpPinDir, LOW);
		analogWrite(m_intPumpPinPWM, intValue);
		m_intStatus = -intValue;
		if (intValue > 0) {
			yellow_led_off();
			red_led_on();
		}	
	}
}

int get_pump_status() {
	return m_intStatus;
}


