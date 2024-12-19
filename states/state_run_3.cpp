#include "state_run_3.h"

#include "..\control\pumps.h"
#include "..\control\pushrod.h"
#include "..\control\servos.h"
#include "..\control\main_motor.h"
#include "..\helpers.h"


double m_dblDepthSP, m_dblAftPitchSP, m_intFwdThrottle, m_intRevThrottle, m_dblDirectionSP;
int m_intFwdDive0Pos, m_intAftRudder0Pos;
unsigned long m_lngRunLength, m_lngTrimEndTP, m_lngFwdEndTP, m_lngRevEndTP, m_lngTimeStart;
boolean blnThrottleZeroed = false;
boolean blnServosZeroed = false;

double dblStartX = 0;

void init_run_3(String strParams) {

	//read the configuration data params
	m_lngRunLength = get_sep_part_dbl(strParams, '|', 0);
	m_intFwdDive0Pos = get_sep_part_dbl(strParams, '|', 1);
	m_intAftRudder0Pos = get_sep_part_dbl(strParams, '|', 2);
	m_intFwdThrottle = get_sep_part_dbl(strParams, '|', 3);

	//set 0 positions of control planes at start of run
	command_servo("SERVOFWDDIVE", m_intFwdDive0Pos, 0);
	delay(100);
	command_servo("SERVOAFTRUDDER", m_intAftRudder0Pos, 0);
	delay(100);

}

void run_start_3(double dblX) {
	
	dblStartX = dblX;

	m_lngTimeStart = millis();

	//initiate fwd run
	commmand_main_motor(m_intFwdThrottle);
}

//only execute this if run has been started - will return true if run is complete
boolean adjust_run_3(double dblX, double dblY,double dblAlt) {

	unsigned long lngTimeELAPSED = millis() - m_lngTimeStart;





	if (lngTimeELAPSED <= m_lngTrimEndTP) {
		//do trim

		/*double dblDirection = dblHeading;
		if (dblDirection > 180) { dblDirection = dblDirection - 360; }
		double dblDirectionError = m_dblDirectionSP - dblDirection;
		int intDirectionOutput = -round(dblDirectionError * 6);
		if (intDirectionOutput > 30) { intDirectionOutput = 30; }
		if (intDirectionOutput < -30) { intDirectionOutput = -30; }
		command_servo("SERVOAFTRUDDER", m_intAftRudder0Pos + intDirectionOutput, intDirectionOutput);*/

		//zig zag	
		//int intPeriod = 6000;
		//int intMaxOffset = 40;
		//float fltPos = (lngTimeELAPSED % m_intZigZagPeriod);
		//float fltqq = fltPos / (m_intZigZagPeriod / (intMaxOffset * 4));
		//if (fltqq > intMaxOffset * 2) { fltqq = (intMaxOffset * 4) - fltqq; }
		//int intDirectionOutput = round(fltqq - intMaxOffset);
		//command_servo("SERVOAFTRUDDER", m_intAftRudder0Pos + intDirectionOutput, intDirectionOutput);

		int intOffset = 40;
		float fltRemainder = (lngTimeELAPSED % m_intZigZagPeriod);
		float fltHalfPeriod = m_intZigZagPeriod / 2.00;
		if (fltRemainder <= fltHalfPeriod) {
			command_servo("SERVOAFTRUDDER", m_intAftRudder0Pos + intOffset, intOffset);
		}
		else {
			command_servo("SERVOAFTRUDDER", m_intAftRudder0Pos - intOffset, -intOffset);
		}


		//double dblPitchError = m_dblAftPitchSP - dblPitch;
		//int intPitchOutput = round(dblPitchError * 2);
		//if (intPitchOutput > 40) { intPitchOutput = 40; }
		//if (intPitchOutput < -40) { intPitchOutput = -40; }
		//command_servo("SERVOAFTDIVE", m_intAftPitch0Pos + intPitchOutput, intPitchOutput);

		//double dblDepthError = m_dblDepthSP - get_depth();
		//int intDepthOutput = -round(dblDepthError * 300); //increased from 200 on 4/7/2023
		//if (intDepthOutput > 40) { intDepthOutput = 40; }
		//if (intDepthOutput < -40) { intDepthOutput = -40; }
		//command_servo("SERVOFWDDIVE", m_intFwdDive0Pos + intDepthOutput, intDepthOutput);

		return false;
	}
	else if (lngTimeELAPSED > m_lngTrimEndTP && lngTimeELAPSED <= m_lngFwdEndTP) {
		//fwd run

		if (!blnServosZeroed) {
			//set 0 positions of control planes at start of run
			command_servo("SERVOFWDDIVE", m_intFwdDive0Pos, 0);
			delay(100);
			command_servo("SERVOAFTDIVE", m_intAftPitch0Pos, 0);
			delay(100);
			command_servo("SERVOAFTRUDDER", m_intAftRudder0Pos, 0);
			blnServosZeroed = true;
		}


		return false;
	}
	else if (lngTimeELAPSED > m_lngFwdEndTP && lngTimeELAPSED <= m_lngRevEndTP) {
		//reverse run

		//if changing from forward to reverse - make sure throttle is zeroed
		if (!blnThrottleZeroed) {
			//need to zero throttle before we can change from fwd to reverse thrust
			commmand_main_motor(0);
			delay(200);
			blnThrottleZeroed = true;
		}

		//continue with reverse run
		commmand_main_motor(m_intRevThrottle);

		return false;
	}
	else {

		//run complete - turn motor off
		commmand_main_motor(0);

		return true;
	}

}







