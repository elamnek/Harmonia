// 
// 
// 

#include <Wire.h>
#include "power_sensor.h"
#include <DFRobot_INA219.h>

DFRobot_INA219_IIC ina219(&Wire, INA219_I2C_ADDRESS4);

// Revise the following two paramters according to actual reading of the INA219 and the multimeter
// for linearly calibration
float ina219Reading_mA = 1000;
float extMeterReading_mA = 1000;

String init_powersensor() {

    delay(1000);

    if (ina219.begin() != true) {
        return "ERROR: ina219 power sensor failed to initialise";
    }
    ina219.linearCalibrate(ina219Reading_mA, extMeterReading_mA);

    return "";

}

float get_bus_voltage() {
    return ina219.getBusVoltage_V();
}
float get_shunt_voltage() {
    return ina219.getShuntVoltage_mV();
}
float get_current_mA() {
    return ina219.getCurrent_mA();
}
float get_power_mW() {
    return ina219.getPower_mW();
}



