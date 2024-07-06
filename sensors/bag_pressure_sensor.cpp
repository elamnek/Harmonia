/*

link to the libraries for the adafruit pressure sensor:
https://github.com/adafruit/Adafruit_MPRLS

*/

#include <Adafruit_MPRLS.h>
#include "bag_pressure_sensor.h"

//pressure sensor for closed loop with air bag
Adafruit_MPRLS mpr = Adafruit_MPRLS(-1, -1);
boolean blnMPR_OK = false;


String init_bagpressuresensor() {

	if (!mpr.begin()) {
		return "ERROR: failed to communicate with MPRLS sensor, check wiring?";
	}

	return "";
}

//returns bag pressure in hPa
float get_bagpressure() {
	return mpr.readPressure();
}
