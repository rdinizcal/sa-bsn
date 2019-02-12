#include "ThermometerModule.hpp"

int32_t main(int32_t argc, char **argv) {
    ThermometerModule sensor(argc, argv);
    
    return sensor.runModule();
}