// 
// 
// 

#include "RTC.h"
#include "..\helpers.h"
#include <GravityRtc.h>

//note rtc on mega need to use ports D20 and D21 by default - so connect the two SDA and SCL wires to these ports
GravityRtc rtc;     //RTC Initialization


void init_rtc() {
	rtc.setup();
	//Set the RTC time automatically: Calibrate RTC time by your computer time
	//rtc.adjustRtc(F(__DATE__), F(__TIME__));	
	
}

void set_rtc_time(String strDateTimeFromPC) {

    int intYear = get_sep_part(strDateTimeFromPC, '|', 0);
    int intMonth = get_sep_part(strDateTimeFromPC, '|', 1);
    int intDay = get_sep_part(strDateTimeFromPC, '|', 2);
    int intHour = get_sep_part(strDateTimeFromPC, '|', 3);
    int intMinute = get_sep_part(strDateTimeFromPC, '|', 4);
    int intSecond = get_sep_part(strDateTimeFromPC, '|', 5);

    rtc.adjustRtc(intYear, intMonth, intDay, 1, intHour, intMinute, intSecond);
	//rtc.adjustRtc(2017, 6, 19, 1, 12, 7, 0);  //Set time: 2017/6/19, Monday, 12:07:00
}

String get_rtc_time() {

	rtc.read();
	
	String strTime = String(rtc.hour) +":"+ String(rtc.minute) + ":" + String(rtc.second) + " " + String(rtc.day) + "/" + String(rtc.month) + "/" + String(rtc.year);

	return strTime;

}

String get_rtc_time_millis() {

	rtc.read();

	String strTime = String(millis()) + "#" + String(rtc.hour) + ":" + String(rtc.minute) + ":" + String(rtc.second) + " " + String(rtc.day) + "/" + String(rtc.month) + "/" + String(rtc.year);

	return strTime;

}


