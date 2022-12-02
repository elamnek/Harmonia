// 
// 
// 

#include "GroveWaterSensor.h"

int m_intFwdWaterSensorPin = 49;
int m_intAftWaterSensorPin = 33;

void init_watersensor() {

	pinMode(m_intFwdWaterSensorPin, INPUT);
	pinMode(m_intAftWaterSensorPin, INPUT);

}

boolean Leak() {

	if (digitalRead(m_intFwdWaterSensorPin) == 0) {
		//|| digitalRead(m_intAftWaterSensorPin) == 0

		return true;	
	}
	return false;
}
