// links to libraries/examples:
// https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
// 

#include "IMU.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "..\comms\rf_comms.h"


Adafruit_BNO055 bno = Adafruit_BNO055(55);

unsigned long m_lngLastRead = 0;
float m_fltOrientation_x = 0.0;
float m_fltOrientation_y = 0.0;
float m_fltOrientation_z = 0.0;


String init_imu() {

    if (!bno.begin()) {
        return "ERROR: IMU sensor failed to initialise";
    }

    delay(1000);

    bno.setExtCrystalUse(true);

    m_lngLastRead - millis();

    return "";

}

void read_imu() {

    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

    m_fltOrientation_x = event.orientation.x;
    m_fltOrientation_y = event.orientation.y;
    m_fltOrientation_z = event.orientation.z;

}

float get_imuorientation_x() {
    return m_fltOrientation_x;
}
float get_imuorientation_y() {
    return m_fltOrientation_y;
}
float get_imuorientation_z() {
    return m_fltOrientation_z;
}

 /*float get_imuorientation_x(boolean blnChkRead) {
     if (blnChkRead) { check_read(); }
     return m_fltOrientation_x;
 }
 float get_imuorientation_y(boolean blnChkRead) {
     if (blnChkRead) { check_read(); }
     return m_fltOrientation_y;
 }
 float get_imuorientation_z(boolean blnChkRead) {
     if (blnChkRead) { check_read(); }
     return m_fltOrientation_z;
 }*/

 

 void check_read() {

     unsigned long lngTimeSinceLastRead = millis() - m_lngLastRead;
     if (lngTimeSinceLastRead > 200) {
         //do a new read

         imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
         m_fltOrientation_x = euler.x();
         m_fltOrientation_y = euler.y();
         m_fltOrientation_z = euler.z();

         send_rf_comm(String(m_fltOrientation_x) + "," + String(m_fltOrientation_y) + "," + String(m_fltOrientation_z));


         /*sensors_event_t event;
         bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
         m_fltOrientation_x = event.orientation.x;
         m_fltOrientation_y = event.orientation.y;
         m_fltOrientation_z = event.orientation.z;*/


         m_lngLastRead = millis();
     } 
 
}
 

