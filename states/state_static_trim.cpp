// 
// 
// 

#include "state_static_trim.h"

//#include "..\sensors\pressure_sensor.h"
//#include "..\sensors\IMU.h"
#include "..\control\pumps.h"
#include "..\control\pushrod.h"
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BNO055.h>


// this function must be non-blocking (no pauses or time delays)
// Rate controls the submarine depth by adjusting the ballast
void adjust_depth(t_control_error depthError){
	float cmd = 0.0;
	cmd = K_P_DEPTH*depthError.err + K_I_DEPTH*depthError.errInt + K_D_DEPTH*depthError.errDer;
	cmd = sgn(cmd)*max(MIN_DEPTH_CMD, abs(cmd));
	if (cmd>=0) {
		command_pump("DEFLATE", (int)constrain(abs(cmd),0,255));
	}
	else {
		command_pump("INFLATE", (int)constrain(abs(cmd),0,255));
	}
}

// this function must be non-blocking (no pauses or time delays)
// Rate controls the position of the battery to shift the CG to achieve trim
void adjust_static_trim(t_control_error trimError){
	float cmd = 0.0;
	cmd = K_P_STAT_PITCH*trimError.err + K_I_STAT_PITCH*trimError.errInt + K_D_STAT_PITCH*trimError.errDer;
	cmd = sgn(cmd)*max(MIN_STATIC_PITCH_CMD, abs(cmd));
	
	if (cmd>0) {
		command_pushrod("FORWARD", (int)(constrain(abs(cmd),0,100));
	}
	else {
		command_pushrod("REVERSE", (int)(constrain(abs(cmd),0,100));
	}
}

// Based on error passed and the timestep from the past values update the error controls
void update_error(float newErr, float dt, t_control_error* errUpdated){
	errUpdated->errDer = (newErr - errUpdated->err)/dt;
	errUpdated->errInt += newErr*dt;
	errUpdated->err = newErr;
}



/*
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


}*/


