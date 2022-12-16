// 
// 
// 

#include "pressure_sensor.h"
#include <MS5837.h>

MS5837 sensor;

String init_presssuresensor(int intFluidDensity) {

    while (!sensor.init()) {
        return "ERROR: bluerobotics pressure sensor failed to initialise";
    }

    // .init sets the sensor model for us but we can override it if required.
    // Uncomment the next line to force the sensor model to the MS5837_30BA.
    //sensor.setModel(MS5837::MS5837_30BA);

    sensor.setFluidDensity(intFluidDensity); // kg/m^3 (freshwater, 1029 for seawater)
    //sensor.setFluidDensity(1000);

    return "";

}

//returns depth in meters
float get_depth() {
    sensor.read();
    return sensor.depth();
}

//returns pressure in mbars
float get_pressure() {
    sensor.read();
    return sensor.pressure();
}

//returns temp in deg C
float get_temperature() {
    sensor.read();
    return sensor.temperature();
}

//returns altitude in m above SL
float get_altitude() {
    sensor.read();
    return sensor.altitude();
}


