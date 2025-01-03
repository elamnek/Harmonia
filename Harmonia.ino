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
//#include "sensors\DVL.h"
#include "sensors\depth_sensor.h"
#include "sensors\bag_pressure_sensor.h"
#include "sensors\power_sensor.h"
#include "comms\rf_comms.h"
#include "comms\LEDs.h"
//#include "data\sdcard.h"
#include "helpers.h"

#define serialDVL Serial2

//variables for dead reckoning report
char m_bufWRPChecksum[3];
String m_strWRPData;
uint8_t m_intWRPCRC8;
double m_dblX, m_dblY, m_dblZ, m_dblRoll, m_dblPitch, m_dblYaw, m_dblStdDev;
int m_intDVLStatus;

//variable for velocity report
char m_bufWRZChecksum[3];
String m_strWRZData;
uint8_t m_intWRZCRC8;
double m_dblvx, m_dblvy, m_dblvz, m_dblAlt;
char m_charValid;

static const uint8_t lookup_table[256] = {
	0x00U,0x07U,0x0EU,0x09U,0x1CU,0x1BU,0x12U,0x15U,
	0x38U,0x3FU,0x36U,0x31U,0x24U,0x23U,0x2AU,0x2DU,
	0x70U,0x77U,0x7EU,0x79U,0x6CU,0x6BU,0x62U,0x65U,
	0x48U,0x4FU,0x46U,0x41U,0x54U,0x53U,0x5AU,0x5DU,
	0xE0U,0xE7U,0xEEU,0xE9U,0xFCU,0xFBU,0xF2U,0xF5U,
	0xD8U,0xDFU,0xD6U,0xD1U,0xC4U,0xC3U,0xCAU,0xCDU,
	0x90U,0x97U,0x9EU,0x99U,0x8CU,0x8BU,0x82U,0x85U,
	0xA8U,0xAFU,0xA6U,0xA1U,0xB4U,0xB3U,0xBAU,0xBDU,
	0xC7U,0xC0U,0xC9U,0xCEU,0xDBU,0xDCU,0xD5U,0xD2U,
	0xFFU,0xF8U,0xF1U,0xF6U,0xE3U,0xE4U,0xEDU,0xEAU,
	0xB7U,0xB0U,0xB9U,0xBEU,0xABU,0xACU,0xA5U,0xA2U,
	0x8FU,0x88U,0x81U,0x86U,0x93U,0x94U,0x9DU,0x9AU,
	0x27U,0x20U,0x29U,0x2EU,0x3BU,0x3CU,0x35U,0x32U,
	0x1FU,0x18U,0x11U,0x16U,0x03U,0x04U,0x0DU,0x0AU,
	0x57U,0x50U,0x59U,0x5EU,0x4BU,0x4CU,0x45U,0x42U,
	0x6FU,0x68U,0x61U,0x66U,0x73U,0x74U,0x7DU,0x7AU,
	0x89U,0x8EU,0x87U,0x80U,0x95U,0x92U,0x9BU,0x9CU,
	0xB1U,0xB6U,0xBFU,0xB8U,0xADU,0xAAU,0xA3U,0xA4U,
	0xF9U,0xFEU,0xF7U,0xF0U,0xE5U,0xE2U,0xEBU,0xECU,
	0xC1U,0xC6U,0xCFU,0xC8U,0xDDU,0xDAU,0xD3U,0xD4U,
	0x69U,0x6EU,0x67U,0x60U,0x75U,0x72U,0x7BU,0x7CU,
	0x51U,0x56U,0x5FU,0x58U,0x4DU,0x4AU,0x43U,0x44U,
	0x19U,0x1EU,0x17U,0x10U,0x05U,0x02U,0x0BU,0x0CU,
	0x21U,0x26U,0x2FU,0x28U,0x3DU,0x3AU,0x33U,0x34U,
	0x4EU,0x49U,0x40U,0x47U,0x52U,0x55U,0x5CU,0x5BU,
	0x76U,0x71U,0x78U,0x7FU,0x6AU,0x6DU,0x64U,0x63U,
	0x3EU,0x39U,0x30U,0x37U,0x22U,0x25U,0x2CU,0x2BU,
	0x06U,0x01U,0x08U,0x0FU,0x1AU,0x1DU,0x14U,0x13U,
	0xAEU,0xA9U,0xA0U,0xA7U,0xB2U,0xB5U,0xBCU,0xBBU,
	0x96U,0x91U,0x98U,0x9FU,0x8AU,0x8DU,0x84U,0x83U,
	0xDEU,0xD9U,0xD0U,0xD7U,0xC2U,0xC5U,0xCCU,0xCBU,
	0xE6U,0xE1U,0xE8U,0xEFU,0xFAU,0xFDU,0xF4U,0xF3U,
};
uint8_t crc8Check(uint8_t* message, int message_length) {
	uint8_t checksum = 0;
	while (message_length > 0) {
		checksum = lookup_table[*message ^ checksum];
		message++;
		message_length--;
	}
	return checksum;
}

//return message to flag success of a command
String m_strMessage;





int m_intCounter = 0;

boolean blnReadyToRun = false;

//auto timer2Hz = timer_create_default();
//auto timer4Hz = timer_create_default();

unsigned long m_lngTestTimeStart, m_lngTestLogTime, m_lngCalTimerStart, m_lngSenderTimer;

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
	//init_dvl(); do not turn this back on - it causes bad problems

	serialDVL.begin(115200, SERIAL_8N1);
	serialDVL.print("wcp,3"); //set serial output - all output excluding the deprecated wrx and wrt sentences
	serialDVL.println("wcw\n"); //get protocal version

	//set range mode to be between 0.05 and 3m altitude (improved performance if only a limited range is selected)
	//serialDVL.print("wcs,,,,,0<=1,");

	serialDVL.println("wcv\n");
	delay(3000);

	//m_strMessage = "";

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

	read_dvl();

	//start interupts
	//timer2Hz.every(500, timer2Hz_interrupt);
	//timer4Hz.every(250, timer4Hz_interrupt);
	m_lngSenderTimer = millis();
}

bool timer2Hz_interrupt() {
//bool timer2Hz_interrupt(void*) {
	

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
				"57|" + String(get_dvlvelocity_alt()) + "," +
				"58|" + String(get_maxdvlvelocity()) + "," +
				"59|" + String(get_maxdvlacceleration()) + "," +
				"60|" + String(get_dvlcrashstopdist()) + "," +
				"61|" + String(get_dvlacceleration());
			
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
		"3|" + String(aft_leak_detected()) + ","  +
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
						//"19|" + String(get_waterpressure()) + "," +
						"18|" + String(get_bagpressure()) + "," +
						"21|" + String(get_pump_status()) + "," +
						"22|" + String(get_main_motor_precision_throttle()) + "," +
		                "23|" + String(get_bus_voltage()) + "," +
		                //"24|" + String(get_shunt_voltage()) + "," +
		                "25|" + String(get_current_mA()) + "," +
		                "26|" + String(get_power_mW()) + "," +
		                //"27|" + String(get_dive_rate_2()) + "," + 
		                //"28|" + String(get_depth_setpoint_2()) + "," +
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
	
	//timer2Hz.tick();
	//timer4Hz.tick();

	if (pressure_maxed()) {
		//send_rf_comm("WARNING: max bag pressure reached");
	}


	read_dvl();

	unsigned long lngTimerNow = millis();
	unsigned long lngDiff = lngTimerNow - m_lngSenderTimer;
	if (lngDiff >= 800) {
		timer2Hz_interrupt();
		m_lngSenderTimer = millis();
	}
	

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

		boolean blnRunDone = adjust_run_3(get_dvldeadreckoning_x(), get_dvlvelocity_x());
		if (blnRunDone) {		
			state = MANL;
			//command_pump("INFLATE", 255);
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

void read_dvl() {

	while (serialDVL.available()) {
		if (serialDVL.available() > 0) {
			String strData = serialDVL.readStringUntil('\n');

			if (strData.startsWith("wra")) {
				m_strMessage = "*******************COMMAND SUCCEEDED*************************";
			}
			else if (strData.startsWith("wrn")) {
				m_strMessage = "*******************COMMAND FAILED*************************";
			}
			else if (strData.startsWith("wrp")) {
				m_strMessage = "";
				m_strWRPData = strData;
				//Serial.println(strData);

				//do checksum
				const int intActualDataLength = m_strWRPData.length() - 4;
				unsigned char bufData[intActualDataLength];
				//char bufDataToRead[intActualDataLength];
				for (int i = 0; i <= intActualDataLength; i++) {
					bufData[i] = m_strWRPData.charAt(i);
					//bufDataToRead[i] = m_strWRPData.charAt(i);
					//Serial.println(String((char)bufData[i]));
				}

				m_bufWRPChecksum[0] = m_strWRPData.charAt(m_strWRPData.length() - 3);
				m_bufWRPChecksum[1] = m_strWRPData.charAt(m_strWRPData.length() - 2);

				m_intWRPCRC8 = crc8Check(bufData, intActualDataLength);

				char bufDataToRead[intActualDataLength];
				for (int i = 0; i <= intActualDataLength; i++) {
					bufDataToRead[i] = bufData[i];
				}

				char* strtokIndx;
				strtokIndx = strtok(bufDataToRead, ","); //read report name
				strtokIndx = strtok(NULL, ","); //read time
				strtokIndx = strtok(NULL, ","); //read x
				m_dblX = atof(strtokIndx);
				strtokIndx = strtok(NULL, ","); //read y
				m_dblY = atof(strtokIndx);
				strtokIndx = strtok(NULL, ","); //read z
				m_dblZ = atof(strtokIndx);
				strtokIndx = strtok(NULL, ","); //read stdDev
				m_dblStdDev = atof(strtokIndx);
				strtokIndx = strtok(NULL, ","); //read roll
				m_dblRoll = atof(strtokIndx);
				strtokIndx = strtok(NULL, ","); //read pitch
				m_dblPitch = atof(strtokIndx);
				strtokIndx = strtok(NULL, ","); //read yaw
				m_dblYaw = atof(strtokIndx);
				strtokIndx = strtok(NULL, ","); //read status
				m_intDVLStatus = atoi(strtokIndx);
			}
			else if (strData.startsWith("wrz")) {
				m_strMessage = "";
				m_strWRZData = strData;
				//Serial.println(strData);

				//do checksum
				int intActualDataLength = m_strWRZData.length() - 4;
				unsigned char bufData[intActualDataLength];
				//char bufDataToRead[intActualDataLength];
				for (int i = 0; i <= intActualDataLength; i++) {
					bufData[i] = m_strWRZData.charAt(i);
					//bufDataToRead[i] = m_strWRPData.charAt(i);
					//Serial.println(String((char)bufData[i]));
				}

				m_bufWRZChecksum[0] = m_strWRZData.charAt(m_strWRZData.length() - 3);
				m_bufWRZChecksum[1] = m_strWRZData.charAt(m_strWRZData.length() - 2);

				m_intWRZCRC8 = crc8Check(bufData, intActualDataLength);

				char bufDataToRead[intActualDataLength];
				for (int i = 0; i <= intActualDataLength; i++) {
					bufDataToRead[i] = bufData[i];
				}

				char* strtokIndx;
				strtokIndx = strtok(bufDataToRead, ","); //read report name
				strtokIndx = strtok(NULL, ","); //read velocity x
				m_dblvx = atof(strtokIndx);
				strtokIndx = strtok(NULL, ","); //read velocit y
				m_dblvy = atof(strtokIndx);
				strtokIndx = strtok(NULL, ","); //read velocity z
				m_dblvz = atof(strtokIndx);
				strtokIndx = strtok(NULL, ","); //read status
				m_charValid = atoi(strtokIndx);
				strtokIndx = strtok(NULL, ","); //read altitude
				m_dblAlt = atof(strtokIndx);

			}
		}
	}

}
void start_dvl_deadreckoning() {
	serialDVL.println("wcr\n");
}
void calibrate_dvl() {
	serialDVL.println("wcg\n");
}
void disable_acoustic_dvl() {
	serialDVL.println("wcs,,,n,,,\n");
}
void enable_acoustic_dvl() {
	serialDVL.println("wcs,,,y,,,\n");
}

String CheckMessage() { return m_strMessage; }

//dead reckoning report properties
uint8_t get_wrp_crc8_result() { return m_intWRPCRC8; }
char* get_wrp_checksum() { return m_bufWRPChecksum; }
String get_wrp_data() { return m_strWRPData; }
double get_dvldeadreckoning_x() { return m_dblX; }
double get_dvldeadreckoning_y() { return m_dblY; }
double get_dvldeadreckoning_z() { return m_dblZ; }
double get_dvldeadreckoning_stdDev() { return m_dblStdDev; }
double get_dvldeadreckoning_roll() { return m_dblRoll; }
double get_dvldeadreckoning_pitch() { return m_dblPitch; }
double get_dvldeadreckoning_yaw() { return m_dblYaw; }
int get_dvldeadreckoning_status() { return m_intDVLStatus; }

//velocity report properties
uint8_t get_wrz_crc8_result() { return m_intWRZCRC8; }
char* get_wrz_checksum() { return m_bufWRZChecksum; }
String get_wrz_data() { return m_strWRZData; }
double get_dvlvelocity_x() { return m_dblvx; }
double get_dvlvelocity_y() { return m_dblvy; }
double get_dvlvelocity_z() { return m_dblvz; }
double get_dvlvelocity_alt() { return m_dblAlt; }
char get_dvlvelocity_valid() { return m_charValid; }




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
