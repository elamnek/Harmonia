// 
// 
// 

#include "main_motor_precision.h"
#include "..\sensors\leonardo_sensors.h"

#include <Motoron.h>

MotoronI2C mc;

int m_intThrottlePrecision = 0;

void init_main_motor_precision() {

	mc.reinitialize();    // Bytes: 0x96 0x74
	mc.disableCrc();      // Bytes: 0x8B 0x04 0x7B 0x43
	mc.clearResetFlag();  // Bytes: 0xA9 0x00 0x04

	// By default, the Motoron is configured to stop the motors if
	// it does not get a motor control command for 1500 ms.  You
	// can uncomment a line below to adjust this time or disable
	// the timeout feature.
	// mc.setCommandTimeoutMilliseconds(1000);
	mc.disableCommandTimeout();

	//Configure motor 1
	mc.setMaxAcceleration(1, 300);
	mc.setMaxDeceleration(1, 300);
}

void commmand_main_motor_precision(int intValue) {

	m_intThrottlePrecision = intValue;
	//main motor throttle commands should always use the scale -800 to + 800 where 0 is motor off

	mc.setSpeed(1, intValue);

	send_leonardo_comm(String(intValue));
}

int get_main_motor_precision_throttle() {
	return m_intThrottlePrecision;
}

