// 
// 
// 

#include "state_run.h"

#include "..\sensors\pressure_sensor.h"
#include "..\sensors\IMU.h"
#include "..\control\pumps.h"
#include "..\control\pushrod.h"
#include "..\control\servos.h"
#include "..\control\main_motor.h"
#include "..\helpers.h"

#include <PID_v1.h>
//https://playground.arduino.cc/Code/PIDLibaryBasicExample/

//dive plane PID
double m_dblDivePlaneSetpoint1, m_dblDivePlaneInput1, m_dblDivePlaneOutput1;
PID m_PIDDivePlane1(&m_dblDivePlaneInput1, &m_dblDivePlaneOutput1, &m_dblDivePlaneSetpoint1, 2, 5, 1, DIRECT);

//rudder PID
double m_dblRudderSetpoint1, m_dblRudderInput1, m_dblRudderOutput1;
PID m_PIDRudder1(&m_dblRudderInput1, &m_dblRudderOutput1, &m_dblRudderSetpoint1, 2, 5, 1, DIRECT);

double m_dblRunThrottle;
unsigned long m_lngRunTime,m_lngStartTime;

void init_run(String strParams) {

	m_dblDivePlaneSetpoint1 = get_sep_part_dbl(strParams, '|', 0);
	m_dblRudderSetpoint1 = get_sep_part_dbl(strParams, '|', 1);
	m_dblRunThrottle = get_sep_part_dbl(strParams, '|', 2);
	int intRunTime = get_sep_part(strParams, '|', 3);
	m_lngRunTime = intRunTime * 1000;

}

void run_start() {

	m_lngStartTime = millis();

	//turn the dive PID on
	m_PIDDivePlane1.SetOutputLimits(-50, 50);
	m_PIDDivePlane1.SetMode(AUTOMATIC);

	//turn the rudder PID on
	m_PIDRudder1.SetOutputLimits(-50, 50);
	m_PIDRudder1.SetMode(AUTOMATIC);

	//turn main motor on
	commmand_main_motor(m_dblRunThrottle);

}

//only execute this if run has been started
void adjust_run(float fltHeading) {

	unsigned long lngElapsedTime = millis() - m_lngStartTime;
	if (lngElapsedTime < m_lngRunTime) {
		//continue with run
		
		//adjust dive PID
		m_dblDivePlaneInput1 = get_depth();
		m_PIDDivePlane1.Compute();
		int intOutput = round(m_dblDivePlaneOutput1);
		intOutput = intOutput + 105;
		command_servo("SERVOAFTDIVE", intOutput);

		//adjust rudder PID
		m_dblRudderInput1 = fltHeading;
		m_PIDRudder1.Compute();
		int intOutput = round(m_dblRudderOutput1);
		intOutput = intOutput + 105;
		command_servo("SERVOAFTRUDDER", intOutput);

	} else {
		//stop run

		//turn motor off
		commmand_main_motor(0);
	}

}

