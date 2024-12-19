#include "state_run_3.h"

#include "..\control\pumps.h"
#include "..\control\pushrod.h"
#include "..\control\servos.h"
#include "..\control\main_motor.h"
#include "..\helpers.h"


double m_dblAftPitchSP, m_intFwdThrottle, m_intRevThrottle, m_dblDirectionSP, m_dblK_Y, m_dblK_Z;
int m_intFwdDive0Pos, m_intAftRudder0Pos;
unsigned long m_lngRunLength, m_lngTrimEndTP, m_lngFwdEndTP, m_lngRevEndTP, m_lngTimeStart;
boolean blnThrottleZeroed = false;
boolean blnServosZeroed = false;
double m_dblStartX = 0;

void init_run_3(String strParams) {

	//read the configuration data params
	m_lngRunLength = get_sep_part_dbl(strParams, '|', 0);
	m_intFwdDive0Pos = get_sep_part_dbl(strParams, '|', 1);
	m_intAftRudder0Pos = get_sep_part_dbl(strParams, '|', 2);
	m_intFwdThrottle = get_sep_part_dbl(strParams, '|', 3);
	m_dblK_Y = get_sep_part_dbl(strParams, '|', 4);
	m_dblK_Z = get_sep_part_dbl(strParams, '|', 5);
	
	//set 0 positions of control planes at start of run
	command_servo("SERVOFWDDIVE", m_intFwdDive0Pos, 0);
	delay(100);
	command_servo("SERVOAFTRUDDER", m_intAftRudder0Pos, 0);
	delay(100);

}

void run_start_3(double dblX) {
	
	m_dblStartX = dblX;

	m_lngTimeStart = millis();

	//initiate fwd run
	commmand_main_motor_precision(m_intFwdThrottle);

}

//only execute this if run has been started - will return true if run is complete
boolean adjust_run_3(double dblX, double dblY,double dblZ) {

	unsigned long lngTimeELAPSED = millis() - m_lngTimeStart;

	double dblDist = dblX - m_dblStartX;

	if (dblDist > m_lngRunLength) { 
		//run complete - turn motor off
		commmand_main_motor_precision(0);
		return true;
	}
	//still in run
	
	//adjust rudder
	double dblYError = dblY;
	int intRudderOutput = round(dblYError * m_dblK_Y);
	if (intRudderOutput > 30) { intRudderOutput = 30; }
	if (intRudderOutput < -30) { intRudderOutput = -30; }
	command_servo("SERVOAFTRUDDER", m_intAftRudder0Pos + intRudderOutput, intRudderOutput);

	//adjust dive
	double dblZError = dblZ;
	int intDiveOutput = round(dblZError * m_dblK_Z);
	if (intDiveOutput > 30) { intDiveOutput = 30; }
	if (intDiveOutput < -30) { intDiveOutput = -30; }
	command_servo("SERVOFWDDIVE", m_intFwdDive0Pos + intDiveOutput, intDiveOutput);

	return false;


}







