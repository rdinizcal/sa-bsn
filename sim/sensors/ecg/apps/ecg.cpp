#include "ECGModule.hpp"

int32_t main(int32_t argc, char **argv) {
    ECGModule sensor(argc, argv);
    
    return sensor.runModule();
}