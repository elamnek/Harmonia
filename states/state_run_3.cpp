#include "state_run_3.h"

#include "..\control\pumps.h"
#include "..\control\pushrod.h"
#include "..\control\servos.h"
#include "..\control\main_motor_precision.h"
#include "..\helpers.h"


double m_intFwdThrottle, m_intRevThrottle;
unsigned long m_lngRunLength, m_lngTimeStart, m_lngTimePrev;
double m_dblStartX = 0;
double m_dblMaxVelocity = 0;
double m_dblMaxAccel = 0;
double m_dblCrashStopDist = 0;
double m_dblPrevVelocity = 0;
double m_dblAccel = 0;

void init_run_3(String strParams) {

	//read the configuration data params
	m_lngRunLength = get_sep_part_dbl(strParams, '|', 0);
	m_intFwdThrottle = get_sep_part_dbl(strParams, '|', 1);

	m_dblStartX = 0;
	m_dblMaxVelocity = 0;
	m_dblMaxAccel = 0;
	m_dblCrashStopDist = 0;
	m_lngTimePrev = 0;
	m_dblPrevVelocity = 0;
	m_dblAccel = 0;
}

void run_start_3(double dblX) {
	
	m_dblStartX = dblX;

	m_lngTimeStart = millis();
	m_lngTimePrev = m_lngTimeStart;

	//initiate fwd run
	commmand_main_motor_precision(m_intFwdThrottle);

}

//only execute this if run has been started - will return true if run is complete
boolean adjust_run_3(double dblX, double dblVelocityX) {

	unsigned long lngTimeNow = millis();
	double dblTimeDiff = (lngTimeNow - m_lngTimePrev)/1000; //convert to seconds
	m_dblAccel = (dblVelocityX - m_dblPrevVelocity) / dblTimeDiff;
	if (m_dblAccel > m_dblMaxAccel) { m_dblMaxAccel = m_dblAccel; }
	if (dblVelocityX > m_dblMaxVelocity) { m_dblMaxVelocity = dblVelocityX; }
	
	double dblDist = dblX - m_dblStartX;
	if (dblDist > m_lngRunLength) { 
		//run complete - turn motor off
		commmand_main_motor_precision(0);
		return true;
	}

	m_dblPrevVelocity = dblVelocityX;
	m_lngTimePrev = lngTimeNow;

	//still in run
	return false;

}

double get_maxdvlvelocity() { return m_dblMaxVelocity; }
double get_dvlacceleration() { return m_dblAccel; }
double get_maxdvlacceleration() { return m_dblMaxAccel; }
double get_dvlcrashstopdist() { return m_dblCrashStopDist; }
