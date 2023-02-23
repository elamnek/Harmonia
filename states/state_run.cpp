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

#include <PID_v1.h>
//https://playground.arduino.cc/Code/PIDLibaryBasicExample/

//dive plane PID
double m_dblDivePlaneSetpoint1, m_dblDivePlaneInput1, m_dblDivePlaneOutput1;
PID m_PIDDivePlane1(&m_dblDivePlaneInput1, &m_dblDivePlaneOutput1, &m_dblDivePlaneSetpoint1, 2, 5, 1, DIRECT);


void init_run() {

	m_PIDDivePlane1.SetOutputLimits(-50, 50);
	m_dblDivePlaneSetpoint1 = get_depth(); //setpoint set to current depth

	//turn the PID on
	m_PIDDivePlane1.SetMode(AUTOMATIC);

	//turn main motor on
	commmand_main_motor(10);

}

void adjust_run() {

	
	m_dblDivePlaneInput1 = get_depth();
	m_PIDDivePlane1.Compute();

	int intOutput = round(m_dblDivePlaneOutput1);
	intOutput = intOutput + 105;

	command_servo("SERVOAFTDIVE", intOutput);


}

