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
//#include <PID_v1.h>
//https://playground.arduino.cc/Code/PIDLibaryBasicExample/


float m_fltDepthSetpoint;

int m_intTimerStart;

boolean m_blnPumpPhase;

float m_fltStartDepth;

float m_fltNeutralDiveRate = 0.5;

int m_intMaxMeasureTime = 5000; //stays fixed
int m_intMaxPumpTime = 5000; //can change depending on dive rate

float m_fltDiveRateCoeff = 5; //the multiplier that is applied to dive rate to get pump time
//for 1cm/s dive rate (fast), pump time will be 1 x 5 = 5s
//for 0.1cm/s dive rate (slow), pump time will be 0.1 x 5 = 0.5s which will saturate to 1s

float m_fltDepthErrorCoeff = 10; //the multiplier that is applied to depth error to get pump time


float m_fltDiveRate = 0;

void init_static_trim(float fltDepthSetpoint) {
	m_fltDepthSetpoint = fltDepthSetpoint;

	m_intTimerStart = millis();
	
	//start with pump on and deflating
	command_pump("DEFLATE", 255);
	m_blnPumpPhase = true;
}


void adjust_depth() {
	
	float fltCurrentDepth = get_depth();
	
	//don't active this process unless sub has dropped below 0.1m
	if (fltCurrentDepth < 0.1) { 
		//sub not below surfaced state - continue deflate
		m_intTimerStart = millis();
		command_pump("DEFLATE", 255);
		m_blnPumpPhase = true;
		return; 
	}

	//sub must now be below 0.1

	int intTimeElapsed = millis() - m_intTimerStart;
	if (m_blnPumpPhase && intTimeElapsed > m_intMaxPumpTime) {
		//in pumping phase and timer has elapsed

		//stop pumping	
		command_pump("DEFLATE", 0);

		//restart timer
		m_intTimerStart = millis();

		//set to pump off phase
		m_blnPumpPhase = false;

		//store start depth
		m_fltStartDepth = fltCurrentDepth;
	
	}
	else if (!m_blnPumpPhase && intTimeElapsed > m_intMaxMeasureTime) {
		//in depth measure phase and timer has elapsed

		//calc dive rate
		float fltDepthChange = fltCurrentDepth - m_fltStartDepth;
		float fltTimeElapsed_s = intTimeElapsed / 1000.00;
		m_fltDiveRate = (fltDepthChange * 100.00) / fltTimeElapsed_s; //units are cm/s  (+ve is diving -ve is climbing)

		//determine what to do in the new pump phase
		if (m_fltDiveRate > -m_fltNeutralDiveRate && m_fltDiveRate < m_fltNeutralDiveRate) {
			//dive/climb rate is very slow (close to neutral bouyancy or on sea bed) - use depth error to adjust pump time
			
			float fltDepthError = m_fltDepthSetpoint - fltCurrentDepth; // -ve error need to inflate, +ve error need to deflate
			if (fltDepthError < 0) {
				command_pump("INFLATE", 255);
				fltDepthError = -fltDepthError;
			}
			else {
				command_pump("DEFLATE", 255);
			}
			m_intMaxPumpTime = round(fltDepthError * m_fltDepthErrorCoeff * 1000);
		
		} else if (m_fltDiveRate >= m_fltNeutralDiveRate) {
			
			//dive rate is fast(ish) set pump to inflate and use dive rate to determine pump time
			command_pump("INFLATE", 255);
			m_intMaxPumpTime = round(m_fltDiveRate * m_fltDiveRateCoeff * 1000);
			
		} else if (m_fltDiveRate <= -m_fltNeutralDiveRate) {

			//climb rate is fast(ish) set pump to deflate and use dive rate to determine pump time
			command_pump("DEFLATE", 255);

			//use dive rate to determine pump time
			m_intMaxPumpTime = -round(m_fltDiveRate * m_fltDiveRateCoeff * 1000);
		}

		//saturate at min and max times
		if (m_intMaxPumpTime > 5000) { m_intMaxPumpTime = 5000; }
		if (m_intMaxPumpTime < 1000) { m_intMaxPumpTime = 1000; }

		//restart timer
		m_intTimerStart = millis();

		//set to pump phase on
		m_blnPumpPhase = true;

	}
}

float get_dive_rate() {
	return m_fltDiveRate;
}




////depth PID
//double m_dblDepthSetpoint, m_dblDepthInput, m_dblDepthOutput;
//PID m_PIDdepth(&m_dblDepthInput, &m_dblDepthOutput, &m_dblDepthSetpoint, 2, 5, 1, DIRECT);
//
////pitch PID
//double m_dblPitchSetpoint, m_dblPitchInput, m_dblPitchOutput;
//PID m_PIDpitch(&m_dblPitchInput, &m_dblPitchOutput, &m_dblPitchSetpoint, 2, 5, 1, DIRECT);
//
//int m_intMode;

//void init_static_trim(double dblDepthSetpoint, int intMode) {
//
//	m_intMode = intMode;
//	m_dblDepthSetpoint = dblDepthSetpoint; //user defined on remote
//	m_dblPitchSetpoint = 0; //horizontal
//
//	if (m_intMode == 0) {
//		//pure PID
//		m_PIDdepth.SetOutputLimits(-255, 255); //just use min and max of pump
//		m_PIDpitch.SetOutputLimits(-255, 255);//just use min and max of pushrod
//	}
//	else {
//		m_PIDdepth.SetOutputLimits(0, 50); //adjust internal pressure (1000-1050 values based on test data from dive testing) - need to add 1000 to output
//		m_PIDpitch.SetOutputLimits(0, 100);//adjust using position of pushrod
//	}
//
//	//turn the PIDs on
//	m_PIDdepth.SetMode(AUTOMATIC);
//	m_PIDpitch.SetMode(AUTOMATIC);
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


