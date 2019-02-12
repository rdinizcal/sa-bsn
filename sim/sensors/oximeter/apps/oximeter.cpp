#include "OximeterModule.hpp"

int32_t main(int32_t argc, char **argv) {
    OximeterModule sensor(argc, argv);
    
    return sensor.runModule();
}