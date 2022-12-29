// 
// 
// 

#include "state_static_trim.h"


//this function must be non-blocking (no pauses or time delays)
//it gets called from the main loop and makes an adjustment to pumps and or pushrod each time it is called based
//on current sensor readings
void adjust_static_trim(float fltStaticTrimDepth) {

	//example calls to sensors:
	float depth = get_depth();
	sensors_vec_t orientation = get_imuorientation();
	float x = orientation.x;

	//example calls to pump
	command_pump("INFLATE", 255); //inflates at full power
	command_pump("DEFLATES", 255); //deflates at full power
	command_pump("DEFLATES", 0); //stops pump

	//example calls to pushrod
	command_pushrod("FORWARD", 100); //pushes battery forward (in fwd direction)(max speed is 255 - but this is very fast)
	command_pushrod("REVERSE", 100); //pushes battery in reverse (towards aft)
	command_pushrod("REVERSE", 0); //stop pushrod

	//if we need to set pump or pushrod going for a set amount of time we need to store current time using millis() function
	//and then on subsequent calls to this function (adjust_static_trim) from main loop, we need to check time diff and determine if pump or pushrod has been going long enough


}
