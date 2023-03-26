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

double m_dblStartDepth, m_dblRunHeading, m_dblRunThrottle;
unsigned long m_lngRunTime,m_lngStartTime;

void init_run(String strParams) {

	m_dblStartDepth = get_sep_part_dbl(strParams, '|', 0);
	m_dblRunHeading = get_sep_part_dbl(strParams, '|', 1);
	m_dblRunThrottle = get_sep_part_dbl(strParams, '|', 2);
	int intRunTime = get_sep_part(strParams, '|', 3);
	m_lngRunTime = intRunTime * 1000;

}

void run_start() {

	m_lngStartTime = millis();

	m_PIDDivePlane1.SetOutputLimits(-50, 50);
	m_dblDivePlaneSetpoint1 = get_depth(); //setpoint set to current depth

	//turn the PID on
	m_PIDDivePlane1.SetMode(AUTOMATIC);

	//turn main motor on
	commmand_main_motor(m_dblRunThrottle);

}

//only execute this if run has been started
void adjust_run() {

	unsigned long lngElapsedTime = millis() - m_lngStartTime;
	if (lngElapsedTime < m_lngRunTime) {
		//continue with run
		m_dblDivePlaneInput1 = get_depth();
		m_PIDDivePlane1.Compute();

		int intOutput = round(m_dblDivePlaneOutput1);
		intOutput = intOutput + 105;

		command_servo("SERVOAFTDIVE", intOutput);

	} else{
		//stop run

		//turn motor off
		commmand_main_motor(0);
	}

}

