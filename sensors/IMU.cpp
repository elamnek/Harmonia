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

float m_fltAcceleration_x = 0.0;
float m_fltAcceleration_y = 0.0;
float m_fltAcceleration_z = 0.0;

//double xPos = 0, yPos = 0, headingVel = 0;

//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
//double ACCEL_VEL_TRANSITION = (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
//double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
//double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees


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

    //get orientation data from IMU
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    m_fltOrientation_x = orientationData.orientation.x;
    m_fltOrientation_y = orientationData.orientation.y;
    m_fltOrientation_z = orientationData.orientation.z;
    
    //get acceleration data from IMU
    sensors_event_t linearAccelData;
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    m_fltAcceleration_x = linearAccelData.acceleration.x;
    m_fltAcceleration_y = linearAccelData.acceleration.y;
    m_fltAcceleration_z = linearAccelData.acceleration.z;

    //calculate position
    //xPos = xPos + ACCEL_POS_TRANSITION * m_fltAcceleration_x;
    //yPos = yPos + ACCEL_POS_TRANSITION * m_fltAcceleration_y;

    //calculate velocity of sensor in the direction it's facing
    //headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);
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
float get_imuacceleration_x() {
    return m_fltAcceleration_x;
}
float get_imuacceleration_y() {
    return m_fltAcceleration_y;
}
float get_imuacceleration_z() {
    return m_fltAcceleration_z;
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
 

