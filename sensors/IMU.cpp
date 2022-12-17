// 
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

//returns pressure in pascals
 float get_imuorientationx() {
    sensors_event_t event;
    bno.getEvent(&event);
    return event.orientation.x;
}

