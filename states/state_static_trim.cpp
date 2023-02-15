// 
// 
// 

#include "state_static_trim.h"

#include "..\sensors\pressure_sensor.h"
#include "..\sensors\IMU.h"
#include "..\sensors\leonardo_sensors.h"
#include "..\control\pumps.h"
#include "..\control\pushrod.h"
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BNO055.h>
#include <PID_v1.h>
//https://playground.arduino.cc/Code/PIDLibaryBasicExample/


//depth PID
double m_dblDepthSetpoint, m_dblDepthInput, m_dblDepthOutput;
PID m_PIDdepth(&m_dblDepthInput, &m_dblDepthOutput, &m_dblDepthSetpoint, 2, 5, 1, DIRECT);

//pitch PID
double m_dblPitchSetpoint, m_dblPitchInput, m_dblPitchOutput;
PID m_PIDpitch(&m_dblPitchInput, &m_dblPitchOutput, &m_dblPitchSetpoint, 2, 5, 1, DIRECT);

void init_static_trim(double dblDepthSetpoint) {

    m_PIDdepth.SetOutputLimits(0, 50); //adjust for min and max of internal pressure (1000-1050 values based on test data from dive testing) - need to add 100 to output
	m_dblDepthSetpoint = dblDepthSetpoint;

	m_PIDpitch.SetOutputLimits(-255,255);//adjust for max and min of pushrod
	//m_PIDpitch.SetSampleTime(1000);
	m_dblPitchSetpoint = 0; //horizontal

	//turn the PIDs on
	m_PIDdepth.SetMode(AUTOMATIC);
	m_PIDpitch.SetMode(AUTOMATIC);
}

boolean adjust_depth() {

	m_dblDepthInput = get_depth();
	m_PIDdepth.Compute();

	float fltPressure = get_leonardo_pressure();
	float fltPressureError = m_dblDepthOutput + 1000 - fltPressure;

	if (fltPressureError > 0) {
		//target pressure is higher than actual pressure - so deflating bag will increase internal pressure
		command_pump("DEFLATE", 255);
	}
	else if (fltPressureError < 0) {
		//target pressure is lower that actual pressure - so inflating bag will decrease internal pressure
		command_pump("INFLATE", 255);
	}
	else {
		command_pump("INFLATE", 0);
	}

	//check actual depth error is within 5cm and return (for use in triggering pitch adjustment
	double dblError = m_dblDepthSetpoint - m_dblDepthInput;
	if (dblError < 0) { dblError = -dblError; }
	if (dblError < 0.05) {
		return true;
	}
	else {
		return false;
	}


}

void adjust_pitch(double dblPitch) {

	//sensors_vec_t o= get_imuorientation();
	m_dblPitchInput = dblPitch;
	m_PIDpitch.Compute();

	int intOutput = round(m_dblPitchOutput);

	if (intOutput > 0) {
		command_pushrod("FORWARD", intOutput);
	} else if (intOutput < 0) {
		intOutput = -intOutput;
		command_pushrod("REVERSE", intOutput);
	} else {
		command_pushrod("FORWARD", 0);
	}

	

}



/*if (fltDepth > fltDepthSetpoint) {
		command_pump("INFLATE", 255);
	}
	else if (fltDepth <= fltDepthSetpoint) {
		command_pump("DEFLATE", 255);
	}*/

// this function must be non-blocking (no pauses or time delays)
// Rate controls the submarine depth by adjusting the ballast
//void adjust_depth(t_control_error depthError){
//	float cmd = 0.0;
//	cmd = K_P_DEPTH*depthError.err + K_I_DEPTH*depthError.errInt + K_D_DEPTH*depthError.errDer;
//	cmd = sgn(cmd)*max(MIN_DEPTH_CMD, abs(cmd));
//	if (cmd>=0) {
//		command_pump("DEFLATE", (int)constrain(abs(cmd),0,255));
//	}
//	else {
//		command_pump("INFLATE", (int)constrain(abs(cmd),0,255));
//	}
//}

// this function must be non-blocking (no pauses or time delays)
// Rate controls the position of the battery to shift the CG to achieve trim
//void adjust_static_trim(t_control_error trimError){
//	float cmd = 0.0;
//	cmd = K_P_STAT_PITCH*trimError.err + K_I_STAT_PITCH*trimError.errInt + K_D_STAT_PITCH*trimError.errDer;
//	cmd = sgn(cmd)*max(MIN_STATIC_PITCH_CMD, abs(cmd));
//	
//	if (cmd>0) {
//		command_pushrod("FORWARD", (int)(constrain(abs(cmd),0,100)));
//	}
//	else {
//		command_pushrod("REVERSE", (int)(constrain(abs(cmd),0,100)));
//	}
//}

// Based on error passed and the timestep from the past values update the error controls
//void update_error(float newErr, float dt, t_control_error* errUpdated){
//	errUpdated->errDer = (newErr - errUpdated->err)/dt;
//	errUpdated->errInt += newErr*dt;
//	errUpdated->err = newErr;
//}

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


