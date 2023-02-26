// 
// 
// 

#include "state_static_trim_2.h"

#include "..\sensors\pressure_sensor.h"
#include "..\sensors\leonardo_sensors.h"
#include "..\control\pumps.h"
#include "..\control\pushrod.h"

int c_maxPushrodPWM = 255;
float c_Kp_pitch_error = 10;

//input params (constants)
const int c_intPumpPWM = 135; //very slow speed
int c_intDataRate = 500; //time interval at which depth and rate of dive is collected
int c_intMinPumpTime = 500;
int c_intMaxPumpTime = 10000;
float c_fltNeutralDiveRate = 1;
float c_fltDepthErrorCoeffDown = 12; //the multiplier that is applied to depth error to get pump time
float c_fltDepthErrorCoeffUp = 20;
float c_fltDiveRateCoeffDown = 2; //the multiplier that is applied to dive rate to get pump time
float c_fltDiveRateCoeffUp = 4;
//note up/down variation in coeffs is to acccount for the fact the inlfating is acting against water pressure wheras deflating is being assisted by it

//example
//for 1cm/s dive rate (slow), pump time will be 1 x 2 = 2s
//for 1cm/s climb rate (slow), pump time will be 1 x 4 = 4s
//for 5cm/s dive rate (fast), pump time will be 5 x 2 = 10s
//for 5cm/s climb rate (fast), pump time will be 5 x 4 = 20s which will saturate to 10s
// 

float c_fltDepthSetpoint;

//variables
int v_intPumpTimerStart;
int v_intPumpTime = 5000;
int v_intDataTimerStart;
float v_fltCurrentDepth;
float v_fltPreviousDepth;
float v_fltDiveRate = 0;

void init_static_trim_2(float fltDepthSetpoint) {
	
	c_fltDepthSetpoint = fltDepthSetpoint;

	v_fltPreviousDepth = get_depth();

	v_intPumpTimerStart = millis();
	v_intDataTimerStart = millis();
	
	//start with pump on and deflating
	command_pump("DEFLATE", c_intPumpPWM);
}


void adjust_depth_2() {

	//first check if the data needs to be updated
	int intDataTimeElapsed = millis() - v_intDataTimerStart;
	if (intDataTimeElapsed > c_intDataRate) {
		
		//get and store new data (current depth and dive rate)
		v_fltCurrentDepth = get_depth();
		float fltDepthChange = v_fltCurrentDepth - v_fltPreviousDepth;
		float fltTimeElapsed_s = intDataTimeElapsed / 1000.00;
		v_fltDiveRate = (fltDepthChange * 100.00) / fltTimeElapsed_s; //units are cm/s  (+ve is diving -ve is climbing)
		
		//reset values for next round
		v_fltPreviousDepth = v_fltCurrentDepth;
		v_intDataTimerStart = millis();
	}

	//don't active this process unless sub has dropped below 0.1m
	//if (m_fltCurrentDepth < 0.1) {
	//	//sub not below surfaced state - continue deflate
	//	m_intPumpTimerStart = millis();
	//	command_pump("DEFLATE", c_intPumpPWM);
	//	return; 
	//}
	//sub must now be below 0.1

	int intPumpTimeElapsed = millis() - v_intPumpTimerStart;
	if (intPumpTimeElapsed > v_intPumpTime) {
		//pump timer has elapsed

		//determine what to do in the next pump phase
		if (v_fltDiveRate > -c_fltNeutralDiveRate && v_fltDiveRate < c_fltNeutralDiveRate) {
			//dive/climb rate is very slow (close to neutral bouyancy or on sea bed) - use depth error to adjust pump time

			float fltDepthError = c_fltDepthSetpoint - v_fltCurrentDepth; // -ve error need to inflate, +ve error need to deflate
			if (fltDepthError < 0) {
				command_pump("INFLATE", c_intPumpPWM);
				v_intPumpTime = -round(fltDepthError * c_fltDepthErrorCoeffUp * 1000);
			}
			else {
				command_pump("DEFLATE", c_intPumpPWM);
				v_intPumpTime = round(fltDepthError * c_fltDepthErrorCoeffDown * 1000);
			}

		}
		else if (v_fltDiveRate >= c_fltNeutralDiveRate) {

			//dive rate is fast(ish) set pump to inflate and use dive rate to determine pump time
			command_pump("INFLATE", c_intPumpPWM);
			v_intPumpTime = round(v_fltDiveRate * c_fltDiveRateCoeffUp * 1000);

		}
		else if (v_fltDiveRate <= -c_fltNeutralDiveRate) {

			//climb rate is fast(ish) set pump to deflate and use dive rate to determine pump time
			command_pump("DEFLATE", c_intPumpPWM);

			//use dive rate to determine pump time
			v_intPumpTime = -round(v_fltDiveRate * c_fltDiveRateCoeffDown * 1000);
		}
	
		v_intPumpTimerStart = millis();
	}

	//saturate at min and max times
	if (v_intPumpTime > c_intMaxPumpTime) { v_intPumpTime = c_intMaxPumpTime; }
	if (v_intPumpTime < c_intMinPumpTime) { v_intPumpTime = c_intMinPumpTime; }

}

float get_dive_rate_2() {
	return v_fltDiveRate;
}
float get_depth_setpoint_2() {
	return c_fltDepthSetpoint;
}

void adjust_pitch_2(float fltPitch) {

	float fltError = fltPitch; //-ve is nose up +ve is nose down

	float fltErrorAbs = fltError; //assume positive
	if (fltError < 0) { fltErrorAbs = -fltError; }

	int intPWM = round(fltErrorAbs * c_Kp_pitch_error);
	if (intPWM > c_maxPushrodPWM) { intPWM = c_maxPushrodPWM; } //saturate

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
