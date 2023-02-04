// 
// 
// 

#include "sdcard.h"
#include <SD.h>
#include <SPI.h>
#include <Servo.h>

SDFile m_SDFile;
int pinCS = 53; // Pin 10 on Arduino Uno Pin 53 for Mega
char m_log_name[] = "HARMONIA.log"; //no more than 8 chars in name

String init_sdcard() {

    pinMode(pinCS, OUTPUT);

    //SD Card Initialization
    if (!SD.begin())
    {
        return "ERROR: SDCard failed to initialise";
    }
    return "";
    
}

void sdcard_save_data(String strDataLine) {
    m_SDFile = SD.open(m_log_name, FILE_WRITE);
    m_SDFile.println(strDataLine);
    m_SDFile.close();
}




