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

double m_dblDepthSP, m_dblAftPitchSP, m_intFwdThrottle, m_intRevThrottle;

int m_intFwdDive0Pos, m_intAftPitch0Pos, m_intAftRudder0Pos;

unsigned long m_lngFwdTime, m_lngRevTime, m_lngTotalTime, m_lngTimeSTART;

boolean blnThrottleZeroed = false;

void init_run_2(String strParams) {


	m_dblDepthSP = get_sep_part_dbl(strParams, '|', 0);
	m_dblAftPitchSP = get_sep_part_dbl(strParams, '|', 1);
	m_intFwdThrottle = get_sep_part(strParams, '|', 2);	
	int intFwdTime = get_sep_part(strParams, '|', 3);	
	m_lngFwdTime = (unsigned long)intFwdTime * 1000;
	m_intRevThrottle = get_sep_part(strParams, '|', 4);	
	int intRevTime = get_sep_part(strParams, '|', 5);
	m_lngRevTime = (unsigned long)intRevTime * 1000;
	m_intFwdDive0Pos = get_sep_part(strParams, '|', 6);
	m_intAftPitch0Pos = get_sep_part(strParams, '|', 7);
	m_intAftRudder0Pos = get_sep_part(strParams, '|', 8);

	m_lngTotalTime = m_lngFwdTime + m_lngRevTime + 200;

	//set 0 positions of control planes at start of run
	command_servo("SERVOFWDDIVE", m_intFwdDive0Pos, 0);
	delay(100);
	command_servo("SERVOAFTDIVE", m_intAftPitch0Pos, 0);
	delay(100);
	command_servo("SERVOAFTRUDDER", m_intAftRudder0Pos, 0);

	init_static_trim_2(m_dblDepthSP, m_dblAftPitchSP);

}

void run_start_2() {

	m_lngTimeSTART = millis();

	//initiate fwd run
	commmand_main_motor(m_intFwdThrottle);

	blnThrottleZeroed = false;

}

//only execute this if run has been started - will return true if run is complete
boolean adjust_run_2(float fltHeading, float fltPitch) {

	unsigned long lngTimeELAPSED = millis() - m_lngTimeSTART;
	if (lngTimeELAPSED <= m_lngFwdTime) {
		//continue with fwd run
		return false;
	}
	else {

		if (!blnThrottleZeroed) {
			//need to zero throttle before we can change from fwd to reverse thrust
			commmand_main_motor(0);
			delay(200);
			blnThrottleZeroed = true;
		}
	
		if (lngTimeELAPSED < m_lngTotalTime) {

			//continue with reverse run
			commmand_main_motor(m_intRevThrottle);
			return false;
		}
		else {
			//stop run

			//turn motor off
			commmand_main_motor(0);

			return true;
		}	
	}
	

}




