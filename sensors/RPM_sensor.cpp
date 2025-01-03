// 
// 
// 

#include "Wire.h"
#include "RPM_sensor.h"

#define rpmAddr 12

String init_rpmsensor() {
	//nothing to do here - wire has already been initialised
}

//rpm
int32_t get_rpm(int intThrottle) {

	if (intThrottle == 0) { return 0; }

	int i = 0;
	int32_t val = 0;
	bool blnSendStop = true;
	Wire.requestFrom(rpmAddr, 4, blnSendStop);
	if (Wire.available() > 0) {
		while (Wire.available() > 0) {  // slave may send less than requested
			val += Wire.read() << (8 * i);
			i++;
		}
		if (intThrottle < 0) { return -val; }
		return val;
	}
	return -1;
}
