// 
// 
// 

#include "LEDs.h"

int m_intRedLEDPin = 23;
int m_intGreenLEDPin = 25;
int m_intOrangeLEDPin = 22;
int m_intYellowLEDPin = 24;

boolean blnOrangeLED_ON = false;

void init_leds() {

	pinMode(m_intRedLEDPin, OUTPUT);
	pinMode(m_intGreenLEDPin, OUTPUT);
	pinMode(m_intOrangeLEDPin, OUTPUT);
	pinMode(m_intYellowLEDPin, OUTPUT);

	//green pin is always on when power is on
	digitalWrite(m_intGreenLEDPin, HIGH);

	//digitalWrite(m_intRedLEDPin, HIGH);delay(1000);digitalWrite(m_intRedLEDPin, LOW);	
	//digitalWrite(m_intOrangeLEDPin, HIGH);delay(1000);digitalWrite(m_intOrangeLEDPin, LOW);
	//digitalWrite(m_intYellowLEDPin, HIGH);delay(1000);digitalWrite(m_intYellowLEDPin, LOW);
}

void red_led_on() {
	digitalWrite(m_intRedLEDPin, HIGH);
}
void red_led_off() {
	digitalWrite(m_intRedLEDPin, LOW);
}
void orange_led_on() {
	digitalWrite(m_intOrangeLEDPin, HIGH);
}
void orange_led_off() {
	digitalWrite(m_intOrangeLEDPin, LOW);
}
void yellow_led_on() {
	digitalWrite(m_intYellowLEDPin, HIGH);
}
void yellow_led_off() {
	digitalWrite(m_intYellowLEDPin, LOW);
}
void toggle_orange_led() {

	if (blnOrangeLED_ON) {
		//LED on - turn off
		digitalWrite(m_intOrangeLEDPin, LOW);
		blnOrangeLED_ON = false;
	}
	else {
		//LED not on - turn on
		digitalWrite(m_intOrangeLEDPin, HIGH);
		blnOrangeLED_ON = true;
	}

}




