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
	//rtc.adjustRtc(F(__DATE__), F(__TIME__));	
	
}

void set_rtc_time(String strDateTimeFromPC) {

    int intYear = getValue(strDateTimeFromPC, '|', 0);
    int intMonth = getValue(strDateTimeFromPC, '|', 1);
    int intDay = getValue(strDateTimeFromPC, '|', 2);
    int intHour = getValue(strDateTimeFromPC, '|', 3);
    int intMinute = getValue(strDateTimeFromPC, '|', 4);
    int intSecond = getValue(strDateTimeFromPC, '|', 5);

    rtc.adjustRtc(intYear, intMonth, intDay, 1, intHour, intMinute, intSecond);
	//rtc.adjustRtc(2017, 6, 19, 1, 12, 7, 0);  //Set time: 2017/6/19, Monday, 12:07:00
}

String get_rtc_time() {

	rtc.read();

	String strTime = String(rtc.hour) +":"+ String(rtc.minute) + ":" + String(rtc.second) + " " + String(rtc.day) + "/" + String(rtc.month) + "/" + String(rtc.year);

	return strTime;

}

// String  var = getValue( StringVar, ',', 2); // if  a,4,D,r  would return D        
int getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length();

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i + 1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]).toInt() : 0;
} 
