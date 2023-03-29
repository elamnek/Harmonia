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
#include "..\states\state_static_trim_2.h"

#include <PID_v1.h>
//https://playground.arduino.cc/Code/PIDLibaryBasicExample/

//dive fwd dive plane PID
double m_dblFwdDivePlaneSetpoint, m_dblFwdDivePlaneInput, m_dblFwdDivePlaneOutput;
PID m_PIDFwdDivePlane(&m_dblFwdDivePlaneInput, &m_dblFwdDivePlaneOutput, &m_dblFwdDivePlaneSetpoint, 2, 5, 1, DIRECT);

//dive aft pitch plane PID
double m_dblAftPitchSetpoint, m_dblAftPitchPlaneInput, m_dblAftPitchPlaneOutput;
PID m_PIDAftPitchPlane(&m_dblAftPitchPlaneInput, &m_dblAftPitchPlaneOutput, &m_dblAftPitchSetpoint, 2, 5, 1, DIRECT);

//rudder PID
double m_dblAftRudderSetpoint, m_dblAftRudderInput, m_dblAftRudderOutput;
PID m_PIDAftRudder(&m_dblAftRudderInput, &m_dblAftRudderOutput, &m_dblAftRudderSetpoint, 2, 5, 1, DIRECT);

int m_intFwdDiveServo0Pos, m_intAftPitchServo0Pos, m_intAftRudderServo0Pos;

double m_dblRunThrottle;
unsigned long m_lngRunTime,m_lngStartTime;

void init_run(String strParams) {

	m_dblFwdDivePlaneSetpoint = get_sep_part_dbl(strParams, '|', 0);
	m_dblAftRudderSetpoint = get_sep_part_dbl(strParams, '|', 1);
	m_dblRunThrottle = get_sep_part_dbl(strParams, '|', 2);
	int intRunTime = get_sep_part(strParams, '|', 3);
	m_lngRunTime = intRunTime * 1000;
	m_intFwdDiveServo0Pos = get_sep_part(strParams, '|', 4);
	m_intAftPitchServo0Pos = get_sep_part(strParams, '|', 5);
	m_intAftRudderServo0Pos = get_sep_part(strParams, '|', 6);

	init_static_trim_2(get_sep_part_dbl(strParams, '|', 0));

}

void run_start() {

	m_lngStartTime = millis();

	//turn the fwd dive PID on
	m_PIDFwdDivePlane.SetOutputLimits(-50, 50);
	m_PIDFwdDivePlane.SetMode(AUTOMATIC);

	//turn the aft pitch PID on
	m_PIDAftPitchPlane.SetOutputLimits(-50, 50);
	m_PIDAftPitchPlane.SetMode(AUTOMATIC);

	//turn the rudder PID on
	m_PIDAftRudder.SetOutputLimits(-50, 50);
	m_PIDAftRudder.SetMode(AUTOMATIC);

	//turn main motor on
	commmand_main_motor(m_dblRunThrottle);

}

//only execute this if run has been started - will return true if run is complete
boolean adjust_run(float fltHeading,float fltPitch) {

	unsigned long lngElapsedTime = millis() - m_lngStartTime;
	if (lngElapsedTime < m_lngRunTime) {
		//continue with run
		
		//adjust fwd dive PID using depth sensor
		m_dblFwdDivePlaneInput = get_depth();
		m_PIDFwdDivePlane.Compute();
		int intOutput = round(m_dblFwdDivePlaneOutput);
		intOutput = intOutput + m_intFwdDiveServo0Pos; //105
		command_servo("SERVOFWDDIVE", intOutput);

		//adjust aft pitch PID using current pitch
		m_dblAftPitchPlaneInput = fltPitch;
		m_PIDAftPitchPlane.Compute();
		intOutput = round(m_dblAftPitchPlaneOutput);
		intOutput = intOutput + m_intAftPitchServo0Pos; //105
		command_servo("SERVOAFTDIVE", intOutput);

		//adjust rudder PID using heading
		m_dblAftRudderInput = fltHeading;
		m_PIDAftRudder.Compute();
		intOutput = round(m_dblAftRudderOutput);
		intOutput = intOutput + m_intAftRudderServo0Pos; //135
		command_servo("SERVOAFTRUDDER", intOutput);

		return false;

	} else {
		//stop run

		//turn motor off
		commmand_main_motor(0);

		return true;
	}

}

