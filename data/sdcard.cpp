// 
// 
// 

#include "sdcard.h"
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include "..\comms\rf_comms.h"

SDFile m_SDFile_1;
SDFile m_SDFile_2;
int pinCS = 53; // Pin 10 on Arduino Uno Pin 53 for Mega
char m_log_name_1[] = "ONEHZ.log"; //no more than 8 chars in name
char m_log_name_2[] = "FOURHZ.log";

String init_sdcard() {

    pinMode(pinCS, OUTPUT);

    //SD Card Initialization
    if (!SD.begin())
    {
        return "ERROR: SDCard failed to initialise";
    }
    return "";
    
}

void sdcard_save_data_1(String strDataLine) {
    m_SDFile_1 = SD.open(m_log_name_1, FILE_WRITE);
    m_SDFile_1.println(strDataLine);
    m_SDFile_1.close();
}

void sdcard_save_data_2(String strDataLine) {
    m_SDFile_2 = SD.open(m_log_name_2, FILE_WRITE);
    m_SDFile_2.println(strDataLine);
    m_SDFile_2.close();
}

void sdcard_record_count() {
    int intRecords = 0;
    m_SDFile_1 = SD.open(m_log_name_1, FILE_READ);
    if (m_SDFile_1) {
        while (m_SDFile_1.available()) {
            String strData = m_SDFile_1.readStringUntil('\r');
            intRecords = intRecords + 1;  
        }
        m_SDFile_1.close();
    }
    else {
        send_rf_comm("Error: could not open HARMONIA log file on SD Card");
    }
    send_rf_comm("records|" + String(intRecords));
}
void sdcard_upload_data() {
    //send_rf_comm("opening log");
    m_SDFile_1 = SD.open(m_log_name_1,FILE_READ);
    if (m_SDFile_1) {
        //send_rf_comm("reading log");
        while (m_SDFile_1.available()) {
            //send_rf_comm("reading record");
            String strData = m_SDFile_1.readStringUntil('\r');
            send_rf_comm(strData);
            delay(100);
        }
        m_SDFile_1.close();
    } else {
        send_rf_comm("Error: could not open HARMONIA log file on SD Card");
    }
}




