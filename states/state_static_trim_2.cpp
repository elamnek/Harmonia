// 
// 
// 

#include "state_static_trim_2.h"

#include "..\sensors\pressure_sensor.h"
#include "..\sensors\leonardo_sensors.h"
#include "..\control\pumps.h"
#include "..\control\pushrod.h"

int m_maxPushrodPWM = 255;
float m_Kp_pitch_error = 10;

//input params
int intPumpPWM = 130;
int m_intDataRate = 500; //time interval at which depth and rate of dive is collected
float m_fltNeutralDiveRate = 1;
float m_fltDepthErrorCoeffDown = 12; //the multiplier that is applied to depth error to get pump time
float m_fltDepthErrorCoeffUp = 12;
float m_fltDiveRateCoeffDown = 5; //the multiplier that is applied to dive rate to get pump time
float m_fltDiveRateCoeffUp = 5;
//for 1cm/s dive rate (fast), pump time will be 1 x 5 = 5s
//for 0.1cm/s dive rate (slow), pump time will be 0.1 x 5 = 0.5s which will saturate to 1s

//variables
float m_fltDepthSetpoint;
int m_intPumpTimerStart;
int m_intMaxPumpTime = 5000;
int m_intDataTimerStart;
float m_fltCurrentDepth;
float m_fltPreviousDepth;
float m_fltDiveRate = 0;
double m_dblDepthSetpoint;

void init_static_trim_2(float fltDepthSetpoint) {
	m_fltDepthSetpoint = fltDepthSetpoint;

	m_fltPreviousDepth = get_depth();

	m_intPumpTimerStart = millis();
	m_intDataTimerStart = millis();
	
	//start with pump on and deflating
	command_pump("DEFLATE", intPumpPWM);
}


void adjust_depth_2() {

	//first check if the data needs to be updated
	int intDataTimeElapsed = millis() - m_intDataTimerStart;
	if (intDataTimeElapsed > m_intDataRate) {
		
		//get and store new data (current depth and dive rate)
		m_fltCurrentDepth = get_depth();
		float fltDepthChange = m_fltCurrentDepth - m_fltPreviousDepth;
		float fltTimeElapsed_s = intDataTimeElapsed / 1000.00;
		m_fltDiveRate = (fltDepthChange * 100.00) / fltTimeElapsed_s; //units are cm/s  (+ve is diving -ve is climbing)
		
		//reset values for next round
		m_fltPreviousDepth = m_fltCurrentDepth;
		m_intDataTimerStart = millis();
	}

	//don't active this process unless sub has dropped below 0.1m
	if (m_fltCurrentDepth < 0.1) {
		//sub not below surfaced state - continue deflate
		m_intPumpTimerStart = millis();
		command_pump("DEFLATE", intPumpPWM);
		return; 
	}

	//sub must now be below 0.1

	int intPumpTimeElapsed = millis() - m_intPumpTimerStart;
	if (intPumpTimeElapsed > m_intMaxPumpTime) {
		//pump timer has elapsed

		//determine what to do in the next pump phase
		if (m_fltDiveRate > -m_fltNeutralDiveRate && m_fltDiveRate < m_fltNeutralDiveRate) {
			//dive/climb rate is very slow (close to neutral bouyancy or on sea bed) - use depth error to adjust pump time

			float fltDepthError = m_fltDepthSetpoint - m_fltCurrentDepth; // -ve error need to inflate, +ve error need to deflate
			if (fltDepthError < 0) {
				command_pump("INFLATE", intPumpPWM);
				m_intMaxPumpTime = -round(fltDepthError * m_fltDepthErrorCoeffUp * 1000);
			}
			else {
				command_pump("DEFLATE", intPumpPWM);
				m_intMaxPumpTime = round(fltDepthError * m_fltDepthErrorCoeffDown * 1000);
			}

		}
		else if (m_fltDiveRate >= m_fltNeutralDiveRate) {

			//dive rate is fast(ish) set pump to inflate and use dive rate to determine pump time
			command_pump("INFLATE", intPumpPWM);
			m_intMaxPumpTime = round(m_fltDiveRate * m_fltDiveRateCoeffUp * 1000);

		}
		else if (m_fltDiveRate <= -m_fltNeutralDiveRate) {

			//climb rate is fast(ish) set pump to deflate and use dive rate to determine pump time
			command_pump("DEFLATE", intPumpPWM);

			//use dive rate to determine pump time
			m_intMaxPumpTime = -round(m_fltDiveRate * m_fltDiveRateCoeffDown * 1000);
		}
	
		m_intPumpTimerStart = millis();
	}

	//saturate at min and max times
	if (m_intMaxPumpTime > 6000) { m_intMaxPumpTime = 6000; }
	if (m_intMaxPumpTime < 1000) { m_intMaxPumpTime = 1000; }

}

float get_dive_rate_2() {
	return m_fltDiveRate;
}

void adjust_pitch_2(float fltPitch) {

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
