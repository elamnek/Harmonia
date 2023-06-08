/*
 Name:		Harmonia.ino
 Created:	9/11/2022 8:06:00 AM
 Author:	eugene lamnek
*/

//installed libraries
#include <DFRobot_INA219.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPL06-007.h>
#include <MS5837.h>
#include <arduino-timer.h>
#include <GravityRtc.h>
#include "Wire.h"
#include <Servo.h>


//harmonia libraries
#include "states\state_manual.h"
#include "states\state_static_trim.h"
#include "states\state_static_trim_2.h"
#include "states\state_dynamic_trim.h"
#include "states\state_run.h"
#include "states\state_run_2.h"
#include "control\pumps.h"
#include "control\main_motor.h"
#include "control\main_motor_precision.h"
#include "control\servos.h"
#include "control\pushrod.h"
#include "sensors\water_sensors.h"
#include "sensors\RTC.h"
#include "sensors\IMU.h"
#include "sensors\leonardo_sensors.h"
#include "sensors\pressure_sensor.h"
#include "sensors\power_sensor.h"
#include "comms\rf_comms.h"
#include "comms\LEDs.h"
#include "data\sdcard.h"
#include "helpers.h"

int m_intCounter = 0;

boolean blnReadyToRun = false;

auto timer2Hz = timer_create_default();
auto timer4Hz = timer_create_default();

unsigned long m_lngTestTimeStart, m_lngTestLogTime, m_lngCalTimerStart;

//FSM states
enum { IDLE, MANUAL, STATIC_TRIM, CALIBRATE_IMU, RUN, SERVO_TEST, ALARM, UPLOAD} state;
//function used to return text description of current state
String  get_state() {
	switch (state) {
	case IDLE: return "IDLE";
	case MANUAL: return "MANUAL";
	case STATIC_TRIM: return "STATIC_TRIM";
	case CALIBRATE_IMU: return "CALIBRATE_IMU";
	case RUN: return "RUN";
	case SERVO_TEST: return "SERVO_TEST";
	case ALARM: return "ALARM";
	case UPLOAD: return "UPLOAD";
	}
}

void setup() {

	//always start in IDLE state
	state = IDLE;
  
	init_rtc();
	init_rf_comms();
	send_rf_comm("Harmonia is awake - stored time is: " + get_rtc_time());

	init_leds();
	init_servos();
	init_pumps();
	init_pushrod();
	init_main_motor_precision();
	init_watersensors();
	init_leonardo_sensors();

	String msg = init_sdcard();
	if (msg.length() > 0) {
		send_rf_comm(msg);
	}
	else {
		send_rf_comm("SDCard OK!!");
	}

	msg = init_imu();
	if (msg.length() > 0) {
		send_rf_comm(msg);
	}
	else {
		send_rf_comm("IMU sensor OK!!");
	}
	
	msg = init_presssuresensor(997);
	if (msg.length() > 0) {
		send_rf_comm(msg);
	}
	else {
		send_rf_comm("water pressure sensor OK!!");
	}

	//this scan reports on addresses of all connected I2C devices
	//I had issues with connecting multiple pressure sensors
	//bluerobotics underwater sensor only support default I2C address
	//so does the internal pressure/temp sensor, which resulted in 2 sensors with same address 
	//- so this is why internal pressure sensor was moved to the aft leonardo uC
	scan_i2c();

	//start interupts
	timer2Hz.every(500, timer2Hz_interrupt);
	timer4Hz.every(250, timer4Hz_interrupt);
				
}

bool timer2Hz_interrupt(void*) {

	
	/// format of the data package is important
	/// each data value needs an associate metadataid (these are defined in the Digital Twin database in a table called dt_data_config
	/// each metadataid needs to have a matching record in the dt_data_config table
	/// the metadataid tells the DT where to insert the data (ie. what table/field combination)
	/// the first data value always has to be the rtc date/time (metadataid=13)
	/// format to use is {metadataid1|data_value1,metadataid2|data_value2,metadataid3|data_value3, etc.}
	/// use curly brackets either end to ensure that entire string is received at remote end and to distinguish from other messages going to remote	
	
	////send_rf_comm(String(m_fltOrientation_x) + "," + String(m_fltOrientation_y) + "," + String(m_fltOrientation_z));
	//unsigned long lngStart = millis();

	read_imu();

	String strData = "{13|" + get_rtc_time() + "," +
						"4|" + get_state() + "," +
						"2|" + String(fwd_leak_detected()) + "," +
						"3|" + String(aft_leak_detected()) + "," +
						"1|" + String(get_depth()) + ","
						"38|" + get_leonardo_rpm_str() + "," +
						"10|" + get_leonardo_pressure_str() + "," +
						"11|" + get_leonardo_temp_str() + "," +
						"14|" + String(get_imuorientation_x()) + "," + //heading
						"15|" + String(get_imuorientation_y()) + "," + //pitch
						"16|" + String(get_imuorientation_z()) + "," + //roll
		                "32|" + String(get_imuacceleration_x()) + "," + 
		                "33|" + String(get_imuacceleration_y()) + "," + 
		                "34|" + String(get_imuacceleration_z()) + "," +
						"17|" + String(get_pushrod_pos()) + "," +
						"19|" + String(get_waterpressure()) + "," +
						"18|" + get_leonardo_bag_pressure_str() + "," +
						"21|" + String(get_pump_status()) + "," +
						"37|" + String(get_main_motor_precision_throttle()) + "," +
		                "23|" + get_leonardo_bus_voltage_str() + "," +
		                "24|" + get_leonardo_shunt_voltage_str() + "," +
		                "25|" + get_leonardo_current_str() + "," +
		                "26|" + get_leonardo_power_str() + "," +
		                "27|" + String(get_dive_rate_2()) + "," + 
		                "28|" + String(get_depth_setpoint_2()) + "," +
		                "35|" + String(millis()) + //milliseconds that uC has been running for
						"}";

	/*unsigned long lngElapsed = millis() - lngStart;
	strData = strData + String(lngElapsed);*/

	//if in alarm state need to flash orange light
	if (state == ALARM) { toggle_orange_led(); }

	m_intCounter = m_intCounter + 1;
	if (m_intCounter < 2) {
		return true;
	}
	else {
		m_intCounter = 0;
	}

	//the following code only executes at 1Hz

	//if in the upload state - we don't want data to be stored or sent to remote
	if (state == UPLOAD) { return true; }

	//every second all operational data needs to be sent to remote (sensors, state, control commands etc.)
	send_rf_comm(strData);

	//save to sdcard
	sdcard_save_data_1(strData);

	return true;
}

bool timer4Hz_interrupt(void*) {

	//this function stored data that needs to be captured at higher fidelity
	//4Hz data doesn't need to be sent to remote display, only stored to SD card (except if in upload mode)

	//if in the upload state - we don't want data to be stored or sent to remote
	if (state == UPLOAD) { return true; }

	String strData = "{13|" + get_rtc_time() + "," +
		              "29|" + get_fwddiveplane_pos() + "," +
		              "30|" + get_aftdiveplane_pos() + "," +
		              "31|" + get_aftrudder_pos() + "," +
		              "35|" + String(millis()) + //milliseconds that uC has been running for
		              "}";

	//may need to comment this if it slows down data transfer
	//send_rf_comm(strData);

	//save to sdcard
	sdcard_save_data_2(strData);

	return true;
}


void loop() {
	
	timer2Hz.tick();
	timer4Hz.tick();

	//check leak sensors and override any state that has been set
	if (fwd_leak_detected() == 1 || aft_leak_detected() == 1) { 
		state = ALARM;		
	}
	else {
		//ignore these if leak detected
		read_leonardo(); //this updates sensor data coming from leonardo	
	}

	//check for new commands coming from desktop remote
	check_rf_comms(); 

	//set state using commands from remote
	String strRemoteCommand = get_remote_command();
	if (state == ALARM) {
		//system in alarm state - can only be changed by command to go to idle state
		if (strRemoteCommand == "IDLE") { state = IDLE; }
		orange_led_on();
	}
	else {
		//system NOT in alarm state - normal state changes allowed
		orange_led_off();
		if (strRemoteCommand == "IDLE") { state = IDLE; }
		if (strRemoteCommand == "MANUAL") { state = MANUAL; }
		if (strRemoteCommand == "CALIBRATE_IMU") { 
			state = CALIBRATE_IMU;
			m_lngCalTimerStart = millis();
			clear_rf_command();
		}
		if (strRemoteCommand == "STATIC_TRIM") {
			state = STATIC_TRIM;
			init_static_trim_2(get_remote_param().toFloat(),0);
			clear_rf_command();
		}
		if (strRemoteCommand == "RUN") { 	
			//String strError = init_imu(); //this will reset heading to 0 so sub needs to be correct direction when run starts
			//if (strError.length() > 0) {
			//	state = IDLE;
			//	send_rf_comm(strError + " RUN aborted");
			//}
			//else {
			//	send_rf_comm("IMU re-started successfully - going into RUN state");
			//	state = RUN;
			//	blnReadyToRun = false;
			//	init_run_2(get_remote_param());
			//	clear_rf_command();
			//}	

			state = RUN;
			blnReadyToRun = false;
			init_run_2(get_remote_param());
			clear_rf_command();

		}

		if (strRemoteCommand == "SERVO_TEST") { 
			state = SERVO_TEST;
			//don't clear the rf command for this state because the parameters are used when the state is exectuted
		}
		if (strRemoteCommand == "ALARM") { state = ALARM; }
		if (strRemoteCommand == "UPLOAD") { 
			state = UPLOAD;
			clear_rf_command();
		}
	}

	//check for command to set time
	if (strRemoteCommand == "TIMESET") {
		set_rtc_time(get_remote_param());
		clear_rf_command();
	}

	//state control	
	switch (state) {
	case IDLE:

		//in idle state need to stop any active operation
		command_pump("INFLATE", 0);
		commmand_main_motor_precision(0);

		break;
	case MANUAL:

		//this checks for a manual command from RF remote and applies it
		apply_manual_command();
		check_pushrod(); //adjusts position of pushrod based on latest setpoint command
		clear_rf_command();

		break;
	case CALIBRATE_IMU:
		
		//check IMU status every second and send the result back to remote
		unsigned long lngCalTimeNow = millis();
		unsigned long lngCalTimeElapsed = lngCalTimeNow - m_lngCalTimerStart;
		if (lngCalTimeElapsed >= 1000) {
			//String strCal = check_imu_calibration();
			send_rf_comm(get_cal());
			m_lngCalTimerStart = millis();
		}

		break;
	case STATIC_TRIM:
	
		adjust_depth_2();
		adjust_pitch_2(get_imuorientation_y());

		break;
	
	case RUN:

		//allow for manual adjustments while running
		apply_manual_command();

		if (!blnReadyToRun) {
			//adjust until trim achieved 
			boolean blnDepthTrim = adjust_depth_2();
			//boolean blnPitchTrim = adjust_pitch_2(get_imuorientation_y()); 
			//&& blnPitchTrim
			if (blnDepthTrim) {
				blnReadyToRun = true;
				command_pushrod("REVERSE", 0);
				delay(200);
				command_pump("DEFLATE", 0);

				run_start_2(get_imuorientation_x());
			}
		}
		else {
			boolean blnRunDone = adjust_run_2(get_imuorientation_x(), get_imuorientation_y());
			if (blnRunDone) {
				
				//just inflate and go into manual state
				//command_pump("INFLATE", 255);
				state = IDLE;
				blnReadyToRun = false;
				
				//when run is complete - maintain depth and pitch until state is changed
				//static_trim_reset();
				//adjust_depth_2();
				//adjust_pitch_2(get_imuorientation_y());
				
			}
		}

		clear_rf_command();

		break;
	case SERVO_TEST:

		//this is a blocking function - the main loop will be halted while the test process runs
		//send_rf_comm("remote param: " + get_remote_param());
		command_servo_test(get_remote_param());
		state = IDLE;
		clear_rf_command();

		break;
	case ALARM:
		//once in alarm state can only exit by user changing to another state via remote software
		
		//inflating will help sub return to surface and also ensure it stops pumping in water if the air bag is ruptured
		command_pump("INFLATE", 255);

		//could also initiate power to surface using motor at high thrust and rudders but this would be a problem in confined space such as tank
		//collision with walls could occur

		//also clear the comms buffer so that any user command to idle state can be read
		clear_rf_command();

		break;
	case UPLOAD:
		//in upload state need to stop any active operation
		command_pump("INFLATE", 0);
		commmand_main_motor_precision(0);

		//send record count to remote:
		sdcard_record_count();

		//run the upload which reads all data in the log and sends it line by line to the remote
		//send_rf_comm("in UPLOAD state");
		sdcard_upload_data();

		//send completion message to remote
		send_rf_comm("upload|done");

		state = IDLE;

		break;
	}
	
	
}

void scan_i2c() {

	byte error, address; //variable for error and I2C address
	int nDevices;

	send_rf_comm("Scanning I2C...");

	nDevices = 0;
	for (address = 1; address < 127; address++)
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0)
		{
			nDevices++;
			String strMsg = "I2C device found at address 0x";
			if (address < 16){ strMsg = strMsg + "0";}
			strMsg = strMsg + String(address, HEX);
			send_rf_comm(strMsg);
			
		}
		else if (error == 4)
		{
			String strMsg = "Unknown error at address 0x";
			if (address < 16) { strMsg = strMsg + "0"; }
			strMsg = strMsg + String(address, HEX);
			send_rf_comm(strMsg);
		}
	}
	if (nDevices == 0)
		send_rf_comm("No I2C devices found");
	else
		send_rf_comm("done");


}


//dead car bodies

//update pitch angle and depth distance
	/*depth_distance = get_depth();
	subOrientation = get_imuorientation();
	pitchAngle = subOrientation.x;*/

//update_error((float)(depthTargetDistance-depth_distance), dt, &depthError);
		//update_error((float)(-pitchAngle), dt, &trimError);
		//
		//// Adjust the depth until the error is with in accpetable margin and has slowed to a near stop
		//if ((abs(depthError.err/ depthTargetDistance)>0.05) || (abs(depthError.errDer)>0.01)) {
		//	adjust_depth(depthError);
		//	trimmed_state &= ~(DEPTH_TRIMMED);
		//}
		//else {
		//	trimmed_state |= DEPTH_TRIMMED;
		//}
		//
		//// Adjust the pitch angle until error is with in accpetable margin and has slowed to a near stop
		//if ((abs(pitchError.err)>0.1) || (abs(pitchError.errDer)>0.01)){
		//	adjust_static_trim(pitchError);
		//	trimmed_state &= ~(PITCH_TRIMMED);
		//}
		//else {
		//	trimmed_state |= PITCH_TRIMMED;
		//}
		//
		//if (trimmed_state == (DEPTH_TRIMMED | PITCH_TRIMMED)) {
		//	command_pump("INFLATE", 0);
		//	state = DYNAMIC_TRIM;
		//}



		//this is a non-blocking function that checks sensors and makes adjustments to trim
		//must be non-blocking so that main loop is always running and checking for leaks or new commands from remote
		//adjust_static_trim(m_fltStaticTrimDepth);

//check for state change and send to remote - move this 2 the 1s timer event
	//if (intStartState != state) {
	//	//note errors occur at remote end if we send a message via RF serial every iteration of the loop - so need to
	//	//only send when necessary
	//	send_rf_comm("STATE=" + String(state));
	//}

//void ProgramESC() {
//
//	//Motor.write(0);//set key??
//	//digitalWrite(m_intMotorPinPWM, LOW);
//	
//	m_servoMainMotor.write(100);//neutral
//	delay(1000);
//	m_servoMainMotor.write(200);//max within 2 seconds of ESC start
//	delay(1500);
//	m_servoMainMotor.write(0);//min
//	delay(1500);
//	m_servoMainMotor.write(100);//neutral
//	delay(1500);
//}

////Check to see if anything is available in the serial receive buffer
//while (Serial.available() > 0)
//{
//
//	//Create a place to hold the incoming message
//	static char message[MAX_MESSAGE_LENGTH];
//	static unsigned int message_pos = 0;
//
//	//Read the next available byte in the serial receive buffer
//	char inByte = Serial.read();
//
//	//Message coming in (check not terminating character) and guard for over message size
//	if (inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH))
//	{
//		//Add the incoming byte to our message
//		message[message_pos] = inByte;
//		message_pos++;
//	}
//	//Full message received...
//	else
//	{
//		//Add null character to string
//		message[message_pos] = '\0';
//
//		//Print the message (or do other things)
//		Serial.println(message);
//
//		String strCommand = String(message);
//		Serial.println("command: " + strCommand);
//
//
//		if (strCommand == "12345") {
//			Serial.println("{inflating: now}");
//			digitalWrite(m_intPumpPinDir, HIGH);
//			analogWrite(m_intPumpPinPWM, 0);
//		}
//		else if (strCommand == "DEF") {
//			Serial.println("deflating");
//			digitalWrite(m_intPumpPinDir, LOW);
//			analogWrite(m_intPumpPinPWM, 255);
//		}
//		else if (strCommand == "FOR") {
//			Serial.println("forward");
//			digitalWrite(m_intPushRodPinDir, HIGH);
//
//			/*if (m_strRemoteParam.length() > 0) {
//				m_strRemoteParam.trim();
//				analogWrite(m_intPushRodPinPWM, m_strRemoteParam.toInt());
//			}
//			else {
//				analogWrite(m_intPushRodPinPWM, 100);
//			}*/
//
//		}
//		else if (strCommand == "REV") {
//			Serial.println("reverse");
//			digitalWrite(m_intPushRodPinDir, LOW);
//			analogWrite(m_intPushRodPinPWM, 100);
//		}
//
//
//
//		//Reset for the next message
//		message_pos = 0;
//	}
//}
