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
int c_intMinPumpPWM = 135;
int c_intMaxPumpPWM = 255;
int c_intDataRate = 50; //time interval at which depth and rate of dive is collected
float c_fltNeutralDiveRate = 1;
float c_fltDepthErrorCoeffDown = 2000; //the multiplier that is applied to depth error and depth to get pump speed
float c_fltDepthErrorCoeffUp = 500;
float c_fltDiveRateCoeffDown = 100; //the multiplier that is applied to dive rate to get pump speed
float c_fltDiveRateCoeffUp = 130;
//note up/down variation in coeffs is to acccount for the fact the inlfating is acting against water pressure wheras deflating is being assisted by it

 
float c_fltDepthSetpoint;

//variables
int v_intDataTimerStart;
float v_fltCurrentDepth;
float v_fltPreviousDepth;
float v_fltDiveRate = 0;

void init_static_trim_2(float fltDepthSetpoint) {
	
	c_fltDepthSetpoint = fltDepthSetpoint;

	v_fltPreviousDepth = get_depth();

	v_intDataTimerStart = millis();
	
	//start with pump on and deflating at min speed
	command_pump("DEFLATE", c_intMinPumpPWM);
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

	//determine pump speed
	if (v_fltDiveRate > -c_fltNeutralDiveRate && v_fltDiveRate < c_fltNeutralDiveRate) {
		//dive/climb rate is very slow (close to neutral bouyancy or on sea bed) - use depth error to adjust speed

		float fltDepthError = c_fltDepthSetpoint - v_fltCurrentDepth; // -ve error need to inflate, +ve error need to deflate
		if (fltDepthError < 0) {

			int intRawPWM = -round(fltDepthError * v_fltCurrentDepth * c_fltDepthErrorCoeffUp);
			int intSatPWM = get_saturated_pwm(intRawPWM);
			command_pump("INFLATE", intSatPWM);
				
		}
		else {
			int intRawPWM = round(fltDepthError * v_fltCurrentDepth * c_fltDepthErrorCoeffDown);
			int intSatPWM = get_saturated_pwm(intRawPWM);
			command_pump("DEFLATE", intSatPWM);
			
		}

	}
	else if (v_fltDiveRate >= c_fltNeutralDiveRate) {

		//dive rate is fast - set pump to inflate and use dive rate to determine pump speed
		int intRawPWM = round(v_fltDiveRate * c_fltDiveRateCoeffUp);
		int intSatPWM = get_saturated_pwm(intRawPWM);
		command_pump("INFLATE", intSatPWM);
			
	}
	else if (v_fltDiveRate <= -c_fltNeutralDiveRate) {

		//climb rate is fast - set pump to deflate and use dive rate to determine speed
		int intRawPWM = -round(v_fltDiveRate * c_fltDiveRateCoeffDown);
		int intSatPWM = get_saturated_pwm(intRawPWM);
		command_pump("DEFLATE", intSatPWM);
	
	}
	
}

int get_saturated_pwm(int intRawPWM) {

	if (intRawPWM > c_intMaxPumpPWM) { return c_intMaxPumpPWM; }
	if (intRawPWM < c_intMinPumpPWM) { return c_intMinPumpPWM; }
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