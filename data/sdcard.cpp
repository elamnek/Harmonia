// 
// 
// 

#include "sdcard.h"
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include "..\comms\rf_comms.h"

SDFile m_SDFile;
int pinCS = 53; // Pin 10 on Arduino Uno Pin 53 for Mega
char m_log_name[] = "HARM.log"; //no more than 8 chars in name

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

void sdcard_record_count() {
    int intRecords = 0;
    m_SDFile = SD.open(m_log_name, FILE_READ);
    if (m_SDFile) {
        while (m_SDFile.available()) {
            String strData = m_SDFile.readStringUntil('\r');
            intRecords = intRecords + 1;  
        }
        m_SDFile.close();
    }
    else {
        send_rf_comm("Error: could not open HARMONIA log file on SD Card");
    }
    send_rf_comm("records|" + String(intRecords));
}
void sdcard_upload_data() {
    //send_rf_comm("opening log");
    m_SDFile = SD.open(m_log_name,FILE_READ);
    if (m_SDFile) {
        //send_rf_comm("reading log");
        while (m_SDFile.available()) {
            //send_rf_comm("reading record");
            String strData = m_SDFile.readStringUntil('\r');
            send_rf_comm(strData);
            delay(100);
        }
        m_SDFile.close();
    } else {
        send_rf_comm("Error: could not open HARMONIA log file on SD Card");
    }
}




