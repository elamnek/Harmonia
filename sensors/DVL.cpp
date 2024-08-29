// 
// 
// 

#include "DVL.h"
#include "..\helpers.h"

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

String init_dvl() {

	serialDVL.begin(115200, SERIAL_8N1);
    
	//serialDVL.print("wcp,3"); //set serial output - all output excluding the deprecated wrx and wrt sentences
	//serialDVL.println("wcw\n"); //get protocal version

	//set range mode to be between 0.05 and 3m altitude (improved performance if only a limited range is selected)
	//serialDVL.print("wcs,,,,,0<=1,");

	//serialDVL.println("wcv\n");
    delay(3000);

    //m_strMessage = "";

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
                int intActualDataLength = m_strWRPData.length() - 4;
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

