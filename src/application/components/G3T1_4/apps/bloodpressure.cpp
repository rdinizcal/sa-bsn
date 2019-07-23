#include "BloodpressureModule.hpp"

int32_t main(int32_t argc, char **argv) {
    BloodpressureModule sensor(argc, argv);

    ros::init(argc, argv, "bloodpressure");

    sensor.setUp();
    sensor.run();

    return 0;
}