#include "BloodpressureModule.hpp"

int32_t main(int32_t argc, char **argv) {
    BloodpressureModule sensor(argc, argv);
    
    return sensor.runModule();
}