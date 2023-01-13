// 
// 
// 

#include "state_manual.h"
#include "..\comms\rf_comms.h"
#include "..\control\pumps.h"
#include "..\control\pushrod.h"
#include "..\control\servos.h"
#include "..\control\main_motor.h"

void apply_manual_command() {

	String strRemoteCommand = get_remote_command();
	String strRemoteParam = get_remote_param();

	if (strRemoteCommand.length() > 0) {
		if (strRemoteCommand == "INFLATE") {
			//send_rf_comm("{inflating: now}");
			command_pump(strRemoteCommand, strRemoteParam.toInt());
		}
		else if (strRemoteCommand == "DEFLATE") {
			//send_rf_comm("deflating");
			command_pump(strRemoteCommand, strRemoteParam.toInt());
		}
		else if (strRemoteCommand == "FORWARD") {
			//send_rf_comm("forward");
			command_pushrod(strRemoteCommand, strRemoteParam.toInt());
		}
		else if (strRemoteCommand == "REVERSE") {
			//send_rf_comm("reverse");
			command_pushrod(strRemoteCommand, strRemoteParam.toInt());
		}
		else if (strRemoteCommand == "PUSHROD") {
			command_pushrod_position(strRemoteParam.toInt());
		}
		else if (strRemoteCommand == "PROPELL") {
			//send_rf_comm("propelling main motor");
			commmand_main_motor(strRemoteParam.toInt());
		}
		else if (strRemoteCommand == "SERVOFWDDIVE") {
			//send_rf_comm("servo forward dive");
			command_servo(strRemoteCommand, strRemoteParam.toInt());
		}
		else if (strRemoteCommand == "SERVOAFTDIVE") {
			//send_rf_comm("servo aft dive");
			command_servo(strRemoteCommand, strRemoteParam.toInt());
		}
		else if (strRemoteCommand == "SERVOAFTRUDDER") {
			//send_rf_comm("servo aft rudder");
			command_servo(strRemoteCommand, strRemoteParam.toInt());
		}
		
	}

}
