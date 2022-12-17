// 
// 
// 

#include "RTC.h"
#include <GravityRtc.h>

//note rtc on mega need to use ports D20 and D21 by default - so connect the two SDA and SCL wires to these ports
GravityRtc rtc;     //RTC Initialization


void init_rtc() {
	rtc.setup();
	//Set the RTC time automatically: Calibrate RTC time by your computer time
	rtc.adjustRtc(F(__DATE__), F(__TIME__));	
	//rtc.adjustRtc(2017, 6, 19, 1, 12, 7, 0);  //Set time: 2017/6/19, Monday, 12:07:00
}

String get_rtctime() {

	rtc.read();

	String strTime = String(rtc.hour) +":"+ String(rtc.minute) + ":" + String(rtc.second) + " " + String(rtc.day) + "/" + String(rtc.month) + "/" + String(rtc.year);

	return strTime;

}
