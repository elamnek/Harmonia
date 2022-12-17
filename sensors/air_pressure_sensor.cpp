// 
// 
// 

#include "air_pressure_sensor.h"
#include <SPL06-007.h>

String init_airpresssuresensor() {

    SPL_init();

    return "";
}

//returns pressure in pascals
double get_airpressure() {
    return get_pressure();
}

