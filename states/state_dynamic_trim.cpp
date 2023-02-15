// 
// 
// 

#include "state_dynamic_trim.h"

#include "..\sensors\pressure_sensor.h"
#include "..\sensors\IMU.h"
#include "..\control\pumps.h"
#include "..\control\pushrod.h"
#include "..\control\servos.h"

#include <PID_v1.h>
//https://playground.arduino.cc/Code/PIDLibaryBasicExample/

//dive plane PID
double m_dblDivePlaneSetpoint, m_dblDivePlaneInput, m_dblDivePlaneOutput;
PID m_PIDDivePlane (&m_dblDivePlaneInput, &m_dblDivePlaneOutput, &m_dblDivePlaneSetpoint, 2, 5, 1, DIRECT);


void init_dynamic_trim() {

	m_PIDDivePlane.SetOutputLimits(-50, 50);
	m_dblDivePlaneSetpoint = 0;

	//turn the PID on
	m_PIDDivePlane.SetMode(AUTOMATIC);
	
}

void adjust_dive_plane(double dblPitch) {

	//sensors_vec_t o= get_imuorientation();
	///m_dblDivePlaneInput = get_altitude();
	
	
	m_dblDivePlaneInput = dblPitch;
	m_PIDDivePlane.Compute();

	int intOutput = round(m_dblDivePlaneOutput);
	intOutput = intOutput + 105;

	command_servo("SERVOAFTDIVE", intOutput);

	
}

