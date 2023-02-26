// 
// 
// 

#include "state_static_trim.h"

#include "..\sensors\pressure_sensor.h"
#include "..\sensors\leonardo_sensors.h"
#include "..\control\pumps.h"
#include "..\control\pushrod.h"
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BNO055.h>
#include <PID_v1.h>
//https://playground.arduino.cc/Code/PIDLibaryBasicExample/


//depth PID
double m_dblDepthSetpoint, m_dblDepthInput, m_dblDepthOutput;
PID m_PIDdepth(&m_dblDepthInput, &m_dblDepthOutput, &m_dblDepthSetpoint, 5, 30, 0, DIRECT);

int m_maxPushrodPWM = 255;
int m_maxPumpPWM = 125;
float m_Kp_pitch_error = 10;
float m_Kp_depth_error = 1000;

void init_static_trim(double dblDepthSetpoint) {

	m_dblDepthSetpoint = 0.5; //user defined on remote
	
	m_PIDdepth.SetOutputLimits(-10, 10); //effective range of ps pump is 130-255 (pwm)
	
	//turn the PID on
	m_PIDdepth.SetMode(AUTOMATIC);
}

void adjust_depth() {

	m_dblDepthInput = get_depth();
	m_PIDdepth.Compute();

	if (m_dblDepthOutput > 0) {
		double dblPWMValue = m_dblDepthOutput + 130; //map to actual
		command_pump("DEFLATE", dblPWMValue);
	}
	else if (m_dblDepthOutput < 0) {
		double dblPWMValue = -m_dblDepthOutput + 130; //map to actual
		command_pump("INFLATE", dblPWMValue);
	}
	else {
		command_pump("INFLATE", 0);
	}
	
}

//void adjust_depth() {
//
//	float fltCurrentDepth = get_depth();
//
//	float fltError = m_dblDepthSetpoint - fltCurrentDepth;
//	float fltErrorAbs = fltError; //assume positive
//	if (fltError < 0) { fltErrorAbs = -fltError; }
//
//	int intPWM = round(fltErrorAbs * m_Kp_depth_error);
//	if (intPWM > m_maxPumpPWM) { intPWM = m_maxPumpPWM; } //saturate
//	
//	if (fltError > 0) {
//		double dblPWMValue = intPWM + 130; //map to actual
//		command_pump("DEFLATE", dblPWMValue);
//	}
//	else if (fltError < 0) {
//		double dblPWMValue = intPWM + 130; //map to actual
//		command_pump("INFLATE", dblPWMValue);
//	}
//	else {
//		command_pump("INFLATE", 0);
//	}
//
//}

void adjust_pitch(float fltPitch) {

	float fltError = fltPitch; //-ve is nose up +ve is nose down

	float fltErrorAbs = fltError; //assume positive
	if (fltError < 0) { fltErrorAbs = -fltError; }

	int intPWM = round(fltErrorAbs * m_Kp_pitch_error);
	if (intPWM > m_maxPushrodPWM) { intPWM = m_maxPushrodPWM; } //saturate

	if (fltError > 0) {
		command_pushrod("REVERSE", intPWM);
	}
	else if (fltError < 0) {
		command_pushrod("FORWARD", intPWM);
	}
	else if (fltError == 0) {
		command_pushrod("REVERSE", 0);
	}



}








//pitch PID
//double m_dblPitchSetpoint, m_dblPitchInput, m_dblPitchOutput;
//PID m_PIDpitch(&m_dblPitchInput, &m_dblPitchOutput, &m_dblPitchSetpoint, 2, 5, 1, DIRECT);

//int m_intMode;

//void init_static_trim(double dblDepthSetpoint) {
//
//	m_dblDepthSetpoint = dblDepthSetpoint; //user defined on remote
//	//m_dblPitchSetpoint = 0; //horizontal
//
//	m_PIDdepth.SetOutputLimits(-125, 125); //effective range of ps pump is 130-255 (pwm)
//	//m_PIDpitch.SetOutputLimits(-255, 255);//just use min and max of pushrod
//
//	//if (m_intMode == 0) {
//	//	//pure PID
//	//	m_PIDdepth.SetOutputLimits(-255, 255); //just use min and max of pump
//	//	m_PIDpitch.SetOutputLimits(-255, 255);//just use min and max of pushrod
//	//}
//	//else {
//	//	m_PIDdepth.SetOutputLimits(0, 50); //adjust internal pressure (1000-1050 values based on test data from dive testing) - need to add 1000 to output
//	//	m_PIDpitch.SetOutputLimits(0, 100);//adjust using position of pushrod
//	//}
//
//	//turn the PIDs on
//	m_PIDdepth.SetMode(AUTOMATIC);
//	//m_PIDpitch.SetMode(AUTOMATIC);
//}

//boolean adjust_depth() {
//
//	m_dblDepthInput = get_depth();
//	m_PIDdepth.Compute();
//
//	if (m_intMode == 0) {
//		if (m_dblDepthOutput > 0) {
//			command_pump("DEFLATE", m_dblDepthOutput);
//		}
//		else if (m_dblDepthOutput < 0) {
//			command_pump("INFLATE", -m_dblDepthOutput);
//		}
//		else {
//			command_pump("INFLATE", 0);
//		}
//	}
//	else {
//		float fltPressure = get_leonardo_pressure();
//		float fltPressureError = m_dblDepthOutput + 1000 - fltPressure;
//		if (fltPressureError > 0) {
//			//target pressure is higher than actual pressure - so deflating bag will increase internal pressure
//			command_pump("DEFLATE", 255);
//		}
//		else if (fltPressureError < 0) {
//			//target pressure is lower that actual pressure - so inflating bag will decrease internal pressure
//			command_pump("INFLATE", 255);
//		}
//		else {
//			command_pump("INFLATE", 0);
//		}
//	}
//	
//	//check actual depth error is within 5cm and return (for use in triggering pitch adjustment)
//	double dblError = m_dblDepthSetpoint - m_dblDepthInput;
//	if (dblError < 0) { dblError = -dblError; }
//	if (dblError < 0.05) {
//		return true;
//	}
//	else {
//		return false;
//	}
//
//}
//
//void adjust_pitch(double dblPitch) {
//
//	m_dblPitchInput = dblPitch;
//	m_PIDpitch.Compute();
//
//	if (m_intMode == 0) {
//		if (m_dblPitchOutput > 0) {
//			command_pushrod("FORWARD", m_dblPitchOutput);
//		}
//		else if (m_dblPitchOutput < 0) {
//			command_pushrod("REVERSE", -m_dblPitchOutput);
//		}
//		else {
//			command_pushrod("FORWARD", 0);
//		}
//	}
//	else {
//
//		int intPos = get_pushrod_pos();
//		double dblPosError = m_dblPitchOutput - intPos;
//
//		if (dblPosError > 0) {
//			command_pushrod("FORWARD", 255);
//		}
//		else if (dblPosError < 0) {
//			command_pushrod("REVERSE", 255);
//		}
//		else {
//			command_pushrod("FORWARD", 0);
//		}
//
//	}
//
//}

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


