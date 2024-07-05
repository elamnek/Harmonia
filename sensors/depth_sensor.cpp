

#include "depth_sensor.h"
#include <MS5837.h>

MS5837 sensor;

float m_fltCachedDepth;

String init_depthsensor(int intFluidDensity) {

    while (!sensor.init()) {
        return "ERROR: bluerobotics pressure sensor failed to initialise";
    }

    // .init sets the sensor model for us but we can override it if required.
    // Uncomment the next line to force the sensor model to the MS5837_30BA.
    //sensor.setModel(MS5837::MS5837_30BA);

    sensor.setFluidDensity(intFluidDensity); // kg/m^3 (freshwater, 1029 for seawater)
    //sensor.setFluidDensity(1000);

    m_fltCachedDepth = 0.0;

    return "";

}

//returns depth in meters
float get_depth() {
    sensor.read();
    m_fltCachedDepth = sensor.depth();
    return m_fltCachedDepth;
}

float get_cached_depth() {
    return m_fltCachedDepth;
}

//returns pressure in mbars
float get_waterpressure() {
    sensor.read();
    return sensor.pressure();
}

//returns temp in deg C
float get_watertemperature() {
    sensor.read();
    return sensor.temperature();
}

//returns altitude in m above SL
float get_altitude() {
    sensor.read();
    return sensor.altitude();
}


