// 
// 
// 

#include "water_sensors.h"

int m_intFwdWaterSensorPin = 40;
int m_intAftWaterSensorPin = 12;

void init_watersensors() {
	pinMode(m_intFwdWaterSensorPin, INPUT);
	pinMode(m_intAftWaterSensorPin, INPUT);
}

boolean leak_detected() {
	//normal state of pins = 1 changes to 0 if water touches sensor  track
	//posibly implement a timer with a min time at 0 to remove chance of false positives
	if (digitalRead(m_intFwdWaterSensorPin) == 0 || digitalRead(m_intAftWaterSensorPin) == 0){
		return true;	
	}
	return false;
}


//analog reads for testing only - not used currently
boolean leak_detected2() {

	if (analogRead(m_intFwdWaterSensorPin) < 80) {
		//|| digitalRead(m_intAftWaterSensorPin) == 0

		return true;
	}
	return false;
}
uint8_t leak_read() {
	return analogRead(m_intFwdWaterSensorPin);
}
