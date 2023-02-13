// links to libraries/examples:
// https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
// 

#include "IMU.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>


Adafruit_BNO055 bno = Adafruit_BNO055(55);

String init_imu() {

    if (!bno.begin()) {
        return "ERROR: IMU sensor failed to initialise";
    }

    delay(1000);

    bno.setExtCrystalUse(true);

    return "";

}


 sensors_vec_t get_imuorientation() {
    sensors_event_t event;
    bno.getEvent(&event);
    return event.orientation;
}

 sensors_event_t get_imu_event() {
    sensors_event_t event;
    bno.getEvent(&event);
    return event;
}
 

