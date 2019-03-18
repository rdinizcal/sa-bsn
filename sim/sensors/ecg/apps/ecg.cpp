#include "ECGModule.hpp"

int32_t main(int32_t argc, char **argv) {
    ECGModule sensor(argc, argv);

    ros::init(argc, argv, "ecg");

    sensor.setUp();
    sensor.run();

    return 0;
}