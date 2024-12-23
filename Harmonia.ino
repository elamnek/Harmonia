/*
 Name:		Harmonia.ino
 Created:	9/11/2022 8:06:00 AM
 Author:	eugene lamnek
*/

//installed libraries
#include <Servo.h>
#include <Adafruit_MPRLS.h>
#include <Motoron.h>
#include <motoron_protocol.h>
#include <PID_v1.h>
#include <DFRobot_INA219.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPL06-007.h>
#include <MS5837.h>
#include <arduino-timer.h>
#include <GravityRtc.h>
//#include "Wire.h"



//harmonia libraries
#include "states\state_manual.h"
//#include "states\state_static_trim.h"
#include "states\state_static_trim_2.h"
#include "states\state_dynamic_trim.h"
#include "states\state_run.h"
#include "states\state_run_2.h"
#include "states\state_run_3.h"
#include "control\pumps.h"
#include "control\main_motor_precision.h"
#include "control\servos.h"
#include "control\pushrod.h"
#include "sensors\water_sensors.h"
#include "sensors\RPM_sensor.h"
#include "sensors\RTC.h"
#include "sensors\IMU.h"
#include "sensors\DVL.h"
#include "sensors\depth_sensor.h"
#include "sensors\bag_pressure_sensor.h"
#include "sensors\power_sensor.h"
#include "comms\rf_comms.h"
#include "comms\LEDs.h"
//#include "data\sdcard.h"
#include "helpers.h"

#define serialDVL Serial2

int m_intCounter = 0;

boolean blnReadyToRun = false;

auto timer2Hz = timer_create_default();
auto timer4Hz = timer_create_default();

unsigned long m_lngTestTimeStart, m_lngTestLogTime, m_lngCalTimerStart;

unsigned long m_lngCycleTimerStart;

String m_strDVLData;

//FSM states
enum { IDLE, REMOTE, MANL,STATIC_TRIM, CALIBRATE_IMU, RUN, SERVO_TEST, ALARM, UPLOAD,CYCLE } state;
//function used to return text description of current state
String  get_state() {
	switch (state) {
	case IDLE: return "IDLE";
	case MANL: return "MANL";
	case REMOTE: return "REMOTE";
	case STATIC_TRIM: return "STATIC_TRIM";
	case CALIBRATE_IMU: return "CALIBRATE_IMU";
	case RUN: return "RUN";
	case SERVO_TEST: return "SERVO_TEST";
	case ALARM: return "ALARM";
	case UPLOAD: return "UPLOAD";
	case CYCLE: return "CYCLE";
	}
}

void setup() {

	//always start in IDLE state
	state = IDLE;
  
	init_rtc();
	init_rf_comms();
	send_rf_comm("Harmonia is awake - stored time is: " + get_rtc_time());

	//init_leds();
	init_servos();
	init_pumps();
	init_pushrod();
	init_main_motor_precision();
	init_watersensors();
	init_dvl();

	serialDVL.begin(115200, SERIAL_8N1);

	String msg = init_powersensor();
	if (msg.length() > 0) {
		send_rf_comm(msg);
	}
	else {
		send_rf_comm("power sensor OK!!");
	}

	/*String msg = init_sdcard();
	if (msg.length() > 0) {
		send_rf_comm(msg);
	}
	else {
		send_rf_comm("SDCard OK!!");
	}*/

	/*msg = init_imu();
	if (msg.length() > 0) {
		send_rf_comm(msg);
	}
	else {
		send_rf_comm("IMU sensor OK!!");
	}*/
	
	msg = init_depthsensor(997);
	if (msg.length() > 0) {
		send_rf_comm(msg);
	}
	else {
		send_rf_comm("water depth sensor OK!!");
	}

	msg = init_bagpressuresensor();
	if (msg.length() > 0) {
		send_rf_comm(msg);
	}
	else {
		send_rf_comm("bag pressure sensor OK!!");
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

	
	if (state == REMOTE) { 
		//if in remote state don't send anything here - all data is send using timer4Hz_interrupt
		return true;
	}

	/// format of the data package is important
	/// each data value needs an associate metadataid (these are defined in the Digital Twin database in a table called dt_data_config
	/// each metadataid needs to have a matching record in the dt_data_config table
	/// the metadataid tells the DT where to insert the data (ie. what table/field combination)
	/// the first data value always has to be the rtc date/time (metadataid=13)
	/// format to use is {metadataid1|data_value1,metadataid2|data_value2,metadataid3|data_value3, etc.}
	/// use curly brackets either end to ensure that entire string is received at remote end and to distinguish from other messages going to remote	
	
	////send_rf_comm(String(m_fltOrientation_x) + "," + String(m_fltOrientation_y) + "," + String(m_fltOrientation_z));
	//unsigned long lngStart = millis();

	//read_imu();


	//check for command failure
	String strMsg = CheckMessage();
	if (strMsg.length() == 0) {
		if (strtoul(get_wrp_checksum(), 0, 16) == get_wrp_crc8_result() && strtoul(get_wrz_checksum(), 0, 16) == get_wrz_crc8_result()) {
			
			m_strDVLData = "46|" + String(get_dvldeadreckoning_x()) + "," +
				"47|" + String(get_dvldeadreckoning_y()) + "," +
				"48|" + String(get_dvldeadreckoning_z()) + "," +
				"49|" + String(get_dvldeadreckoning_roll()) + "," +
				"50|" + String(get_dvldeadreckoning_pitch()) + "," +
				"51|" + String(get_dvldeadreckoning_yaw()) + "," +
				"52|" + String(get_dvldeadreckoning_stdDev()) + "," +
				"53|" + String(get_dvldeadreckoning_status()) + "," +
				"54|" + String(get_dvlvelocity_x()) + "," +
				"55|" + String(get_dvlvelocity_y()) + "," +
				"56|" + String(get_dvlvelocity_z()) + "," +
				"57|" + String(get_dvlvelocity_alt());

		}
		else {
			send_rf_comm("DVL CRC Error!");
		}
	} 
	else {
		send_rf_comm(strMsg);
	}

	String strData = "{13|" + get_rtc_time() + "," +
		               "4|" + get_state() + "," +
						"2|" + String(fwd_leak_detected()) + "," +
						"3|" + String(aft_leak_detected()) + "," +
						"1|" + String(get_depth()) + ","
						"38|" + String(get_rpm(get_main_motor_precision_throttle())) + "," +
		                m_strDVLData + "," +
						//"10|" + get_leonardo_pressure_str() + "," +
						//"11|" + get_leonardo_temp_str() + "," +*/
						//"14|" + String(get_imuorientation_x()) + "," + //heading
						//"15|" + String(get_imuorientation_y()) + "," + //pitch
						//"16|" + String(get_imuorientation_z()) + "," + //roll
		                //"32|" + String(get_imuacceleration_x()) + "," + 
		                //"33|" + String(get_imuacceleration_y()) + "," + 
		                //"34|" + String(get_imuacceleration_z()) + "," +
		                //"45|" + String(get_imucal()) + "," +
						"17|" + String(get_pushrod_pos()) + "," +
						"19|" + String(get_waterpressure()) + "," +
						"18|" + String(get_bagpressure()) + "," +
						"21|" + String(get_pump_status()) + "," +
						"22|" + String(get_main_motor_precision_throttle()) + "," +
		                "23|" + String(get_bus_voltage()) + "," +
		                "24|" + String(get_shunt_voltage()) + "," +
		                "25|" + String(get_current_mA()) + "," +
		                "26|" + String(get_power_mW()) + "," +
		                "27|" + String(get_dive_rate_2()) + "," + 
		                "28|" + String(get_depth_setpoint_2()) + "," +
		                "35|" + String(millis()) + //milliseconds that uC has been running for
						"}";

	/*unsigned long lngElapsed = millis() - lngStart;
	strData = strData + String(lngElapsed);*/

	//if in alarm state need to flash orange light
	//if (state == ALARM) { toggle_orange_led(); }

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
	//sdcard_save_data_1(strData);

	return true;
}

bool timer4Hz_interrupt(void*) {


	if (state == REMOTE) {

		//if in remote state need to send current throttle, rudder and diveplane settings as well as leak information
		String strData = "{4|" + get_state() + "," +
			"2|" + String(fwd_leak_detected()) + "," +
			"3|" + String(aft_leak_detected()) + "," +
			"22|" + String(get_main_motor_precision_throttle()) + "," +
			"23|" + String(get_bus_voltage()) + "," +
			"29|" + get_fwddiveplane_pos() + "," +
			"31|" + get_aftrudder_pos() +
			"}";
		return true;
	}

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
	//sdcard_save_data_2(strData);

	return true;
}


void loop() {
	
	timer2Hz.tick();
	timer4Hz.tick();

	read_dvl();

	//check leak sensors and override any state that has been set
	//if (fwd_leak_detected() == 1 || aft_leak_detected() == 1) { 
	//	state = ALARM;		
	//}
	//else {
	//	//ignore these if leak detected
	//	if (state != REMOTE) {
	//		//read_leonardo(); //this updates sensor data coming from leonardo - don't need this data in remote state	
	//	}
	//}

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
		//system mot in alarm state - normal state changes allowed
		
		orange_led_off();
		if (strRemoteCommand == "IDLE") { state = IDLE; }
		if (strRemoteCommand == "MANL") {state = MANL; }
		if (strRemoteCommand == "REMOTE") { state = REMOTE; }
		if (strRemoteCommand == "CALIBRATE_IMU") { 
			state = CALIBRATE_IMU;
			//m_lngCalTimerStart = millis();
			//clear_rf_command();
		}
		if (strRemoteCommand == "STATIC_TRIM") {
			state = STATIC_TRIM;
			init_static_trim_2(get_remote_param().toFloat(),0);
			send_rf_comm("STATIC_TRIM INIT");
			clear_rf_command();
		}
		if (strRemoteCommand == "RUN") { 	
			
			state = RUN;

			//make sure pump is switched of when run starts
			command_pump("INFLATE", 0);

			init_run_3(get_remote_param());
			run_start_3(get_dvldeadreckoning_x());
			clear_rf_command();
			
		}
		if (strRemoteCommand == "CYCLE") {
			state = CYCLE;
			m_lngCycleTimerStart = millis();
			clear_rf_command();
			send_rf_comm("CYCLE INIT");
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
	case MANL:

		//this checks for a manual command from RF remote and applies it
		apply_manual_command();
		check_pushrod(); //adjusts position of pushrod based on latest setpoint command
		clear_rf_command();

		break;
	case CYCLE:

		check_pushrod(); //adjusts position of pushrod based on latest setpoint command

		unsigned long lngElapsedTime = millis() - m_lngCycleTimerStart;
		if (lngElapsedTime <= 15000) {
			commmand_main_motor_precision(10);
			command_pump("INFLATE", 255);
			command_pushrod_position(0);
			command_servo("SERVOFWDDIVE", 40, 40);
		}
		else if (lngElapsedTime > 15000 && lngElapsedTime <= 35000) {
			command_pushrod_position(20);
			command_servo("SERVOFWDDIVE", 90, 90);
		}
		else if (lngElapsedTime > 35000 && lngElapsedTime <= 45000) {
			command_pushrod_position(40);
			command_servo("SERVOFWDDIVE", 140, 140);
		}
		else if (lngElapsedTime > 45000 && lngElapsedTime <= 55000) {
			command_pushrod_position(60);
			command_pump("DEFLATE", 255);
			commmand_main_motor_precision(0);
			command_servo("SERVOFWDDIVE", 0, 0);
		}
		else if (lngElapsedTime > 55000 && lngElapsedTime <= 65000) {
			command_pushrod_position(80);
			command_servo("SERVOFWDDIVE", 70, 70);
		}
		else if (lngElapsedTime > 65000 && lngElapsedTime <= 75000) {
			command_pushrod_position(100);
			command_servo("SERVOFWDDIVE", 120, 120);
		}
		else {
			command_pump("DEFLATE", 0);
			commmand_main_motor_precision(0);
			command_pushrod_position(0);
			m_lngCycleTimerStart = millis();
			send_rf_comm("repeating");
		}

		break;
	case REMOTE:

		//this checks for a manual command from HANDHELD RF remote and applies it
		apply_manual_command();
		check_pushrod(); //adjusts position of pushrod based on latest setpoint command
		clear_rf_command();

		break;
	case CALIBRATE_IMU:

		send_rf_comm("checking IMU calibration");
		//check_imu_calibration();
		delay(200);
		
		//check IMU status every second and send the result back to remote
		//unsigned long lngCalTimeNow = millis();
		//unsigned long lngCalTimeElapsed = lngCalTimeNow - m_lngCalTimerStart;
		//if (lngCalTimeElapsed >= 1000) {
		//	//String strCal = check_imu_calibration();
		//	send_rf_comm(get_imucalibration());
		//	m_lngCalTimerStart = millis();
		//}

		break;
	case STATIC_TRIM:
	
		adjust_depth_2();
		//adjust_pitch_2(get_imuorientation_y());

		break;
	
	case RUN:

		boolean blnRunDone = adjust_run_3(get_dvldeadreckoning_x(), get_dvldeadreckoning_y(), get_dvldeadreckoning_z());
		if (blnRunDone) {		
			state = MANL;
			command_pump("INFLATE", 255);
		}

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
		//sdcard_record_count();

		//run the upload which reads all data in the log and sends it line by line to the remote
		//send_rf_comm("in UPLOAD state");
		//sdcard_upload_data();

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
