// 
// 
// 

#include "state_run_2.h"

#include "..\sensors\pressure_sensor.h"
#include "..\sensors\IMU.h"
#include "..\control\pumps.h"
#include "..\control\pushrod.h"
#include "..\control\servos.h"
#include "..\control\main_motor.h"
#include "..\helpers.h"
#include "..\states\state_static_trim_2.h"

double m_dblDepthSP, m_dblAftPitchSP, m_intFwdThrottle, m_intRevThrottle, m_dblDirectionSP;

int m_intFwdDive0Pos, m_intAftPitch0Pos, m_intAftRudder0Pos;

unsigned long m_lngTrimEndTP,m_lngFwdEndTP, m_lngRevEndTP, m_lngTimeStart;

boolean blnThrottleZeroed = false;
boolean blnServosZeroed = false;

void init_run_2(String strParams) {


	//read the configuration data params
	m_dblDepthSP = get_sep_part_dbl(strParams, '|', 0);
	m_dblAftPitchSP = get_sep_part_dbl(strParams, '|', 1);
	m_intFwdThrottle = get_sep_part(strParams, '|', 2);	
	int intFwdTime = get_sep_part(strParams, '|', 3);		
	m_intRevThrottle = get_sep_part(strParams, '|', 4);	
	int intRevTime = get_sep_part(strParams, '|', 5);	
	m_intFwdDive0Pos = get_sep_part(strParams, '|', 6);
	m_intAftPitch0Pos = get_sep_part(strParams, '|', 7);
	m_intAftRudder0Pos = get_sep_part(strParams, '|', 8);
	int intTrimTime = get_sep_part(strParams, '|', 9);
	m_dblDirectionSP = get_sep_part_dbl(strParams, '|', 10);
	
	//work out the elapsed time points
	m_lngTrimEndTP = (unsigned long)intTrimTime * 1000;
	m_lngFwdEndTP = m_lngTrimEndTP + ((unsigned long)intFwdTime * 1000);
	m_lngRevEndTP = m_lngFwdEndTP + 200 + ((unsigned long)intRevTime * 1000);//add delay time where motor is set to 0
	
	//set 0 positions of control planes at start of run
	command_servo("SERVOFWDDIVE", m_intFwdDive0Pos, 0);
	delay(100);
	command_servo("SERVOAFTDIVE", m_intAftPitch0Pos, 0);
	delay(100);
	command_servo("SERVOAFTRUDDER", m_intAftRudder0Pos, 0);

	init_static_trim_2(m_dblDepthSP, m_dblAftPitchSP);

}

void static_trim_reset() {
	init_static_trim_2(m_dblDepthSP, m_dblAftPitchSP);
}

void run_start_2() {

	m_lngTimeStart = millis();

	//initiate fwd run
	commmand_main_motor(m_intFwdThrottle);

	blnThrottleZeroed = false;
	blnServosZeroed = false;

}

//only execute this if run has been started - will return true if run is complete
boolean adjust_run_2(double dblHeading, double dblPitch) {

	unsigned long lngTimeELAPSED = millis() - m_lngTimeStart;
	if (lngTimeELAPSED <= m_lngTrimEndTP) {
		//do trim

		double dblDirection = dblHeading;
		if (dblDirection > 180) { dblDirection = dblDirection - 360; }
		double dblDirectionError = m_dblDirectionSP - dblDirection;
		int intDirectionOutput = -round(dblDirectionError * 5);
		if (intDirectionOutput > 30) { intDirectionOutput = 30; }
		if (intDirectionOutput < -30) { intDirectionOutput = -30; }
		command_servo("SERVOAFTRUDDER", m_intAftRudder0Pos + intDirectionOutput, intDirectionOutput);
		
		double dblPitchError = m_dblAftPitchSP - dblPitch;
		int intPitchOutput = round(dblPitchError * 2);
		if (intPitchOutput > 40) { intPitchOutput = 40; }
		if (intPitchOutput < -40) { intPitchOutput = -40; }
		command_servo("SERVOAFTDIVE", m_intAftPitch0Pos + intPitchOutput, intPitchOutput);

		double dblDepthError = m_dblDepthSP - get_depth();
		int intDepthOutput = -round(dblDepthError * 200);
		if (intDepthOutput > 40) { intDepthOutput = 40; }
		if (intDepthOutput < -40) { intDepthOutput = -40; }
		command_servo("SERVOFWDDIVE", m_intFwdDive0Pos + intDepthOutput, intDepthOutput);
	
		return false;
	} else if (lngTimeELAPSED > m_lngTrimEndTP && lngTimeELAPSED <= m_lngFwdEndTP) {
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
	} else if (lngTimeELAPSED > m_lngFwdEndTP && lngTimeELAPSED <= m_lngRevEndTP) {
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
	} else {

		//run complete - turn motor off
		commmand_main_motor(0);

		return true;	
	}

}




