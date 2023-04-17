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

double m_dblAftPitchSP, m_dblDepthSP, m_dblAftRudderSP, m_intThrottleRUN;

int m_intFwdDive0Pos, m_intAftPitch0Pos, m_intAftRudder0Pos;

unsigned long m_lngTimeRUN, m_lngTimeSTART;

void init_run_2(String strParams) {

	m_dblAftPitchSP = 0;

	m_dblDepthSP = get_sep_part_dbl(strParams, '|', 0);
	m_dblAftRudderSP = get_sep_part_dbl(strParams, '|', 1);
	m_intThrottleRUN = get_sep_part(strParams, '|', 2);
	int intTimeRUN = get_sep_part(strParams, '|', 3);
	m_lngTimeRUN = (unsigned long)intTimeRUN * 1000;
	m_intFwdDive0Pos = get_sep_part(strParams, '|', 4);
	m_intAftPitch0Pos = get_sep_part(strParams, '|', 5);
	m_intAftRudder0Pos = get_sep_part(strParams, '|', 6);

	//set 0 positions of control planes at start of run (otherwise they look weird!)
	command_servo("SERVOFWDDIVE", m_intFwdDive0Pos, 0);
	delay(100);
	command_servo("SERVOAFTDIVE", m_intAftPitch0Pos, 0);
	delay(100);
	command_servo("SERVOAFTRUDDER", m_intAftRudder0Pos, 0);

	init_static_trim_2(get_sep_part_dbl(strParams, '|', 0));

}

void run_start_2() {

	m_lngTimeSTART = millis();

	////turn the fwd dive PID on
	//m_PIDFwdDivePlane.SetOutputLimits(-40, 27);
	//m_PIDFwdDivePlane.SetMode(AUTOMATIC);

	////turn the aft pitch PID on
	//m_PIDAftPitchPlane.SetOutputLimits(-40, 40);
	//m_PIDAftPitchPlane.SetMode(AUTOMATIC);

	////turn the rudder PID on
	//m_PIDAftRudder.SetOutputLimits(-45, 45);
	//m_PIDAftRudder.SetMode(AUTOMATIC);

	//turn main motor on
	commmand_main_motor(m_intThrottleRUN);

}

//only execute this if run has been started - will return true if run is complete
boolean adjust_run_2(float fltHeading, float fltPitch) {

	unsigned long lngTimeELAPSED = millis() - m_lngTimeSTART;
	if (lngTimeELAPSED < m_lngTimeRUN) {
		//continue with run

		int intOutput = PropControl(m_dblDepthSP,get_depth(),)
		command_servo("SERVOFWDDIVE", intOutput, intOutput - m_intFwdDiveServo0Pos);

		////adjust fwd dive PID using depth sensor
		//m_dblFwdDivePlaneInput = get_depth();
		//m_PIDFwdDivePlane.Compute();
		//int intOutput = round(m_dblFwdDivePlaneOutput);
		//intOutput = intOutput + m_intFwdDiveServo0Pos; //105
		//command_servo("SERVOFWDDIVE", intOutput, intOutput - m_intFwdDiveServo0Pos);

		////adjust aft pitch PID using current pitch
		//m_dblAftPitchPlaneInput = fltPitch;
		//m_PIDAftPitchPlane.Compute();
		//intOutput = round(m_dblAftPitchPlaneOutput);
		//intOutput = intOutput + m_intAftPitchServo0Pos; //105
		//command_servo("SERVOAFTDIVE", intOutput, intOutput - m_intAftPitchServo0Pos);

		////adjust rudder PID using heading
		////convert heading to direction -180 - +180
		//float fltDirection = fltHeading;
		//if (fltDirection > 180) { fltDirection = fltDirection - 360; }
		//m_dblAftRudderInput = -fltDirection; //negative sign untested
		//m_PIDAftRudder.Compute();
		//intOutput = round(m_dblAftRudderOutput);
		//intOutput = intOutput + m_intAftRudderServo0Pos; //135
		//command_servo("SERVOAFTRUDDER", intOutput, intOutput - m_intAftRudderServo0Pos);

		return false;

	}
	else {
		//stop run

		//turn motor off
		commmand_main_motor(0);

		return true;
	}
	

}
int PropControl(double dblSP,double dblCurrentValue,double dblCoeff,int intMaxValue,int intMinValue){

	double dblError = m_dblFwdDivePlaneSP - get_depth();

	if (intRawPWM > c_intMaxPumpPWM) { return c_intMaxPumpPWM; }
	if (intRawPWM < c_intMinPumpPWM) { return c_intMinPumpPWM; }

}



