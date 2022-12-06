// 
// 
// 

#include "water_sensors.h"

int m_intFwdWaterSensorPin = 40;
int m_intAftWaterSensorPin = 33;

void init_watersensors() {

	pinMode(m_intFwdWaterSensorPin, INPUT);
	pinMode(m_intAftWaterSensorPin, INPUT);

}

boolean leak_detected() {

	if (digitalRead(m_intFwdWaterSensorPin) == 0) {
		//|| digitalRead(m_intAftWaterSensorPin) == 0

		return true;	
	}
	return false;
}
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
