#include "ThermometerModule.hpp"

int32_t main(int32_t argc, char **argv) {
    ThermometerModule sensor(argc, argv);

    ros::init(argc, argv, "thermometer");
    
    sensor.setUp();
    sensor.run();

    return 0;
}