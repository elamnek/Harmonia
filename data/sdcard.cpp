// 
// 
// 

#include "sdcard.h"
#include <SD.h>
#include <SPI.h>
#include <Servo.h>

SDFile m_SDFile;
int pinCS = 53; // Pin 10 on Arduino Uno Pin 53 for Mega
char m_log_name[] = "ATVNAV.csv"; //no more than 8 chars in name

String init_sdcard() {

    pinMode(pinCS, OUTPUT);

    //SD Card Initialization
    if (!SD.begin())
    {
        return "ERROR: SDCard failed to initialise";
    }
    return "";
    
}




