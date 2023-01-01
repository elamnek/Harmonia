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
	errUpdated->errDer = (newErr - errUpdated.err)/dt;
	errUpdated->errInt += newErr*dt;
	errUpdated->err = newErr;
}