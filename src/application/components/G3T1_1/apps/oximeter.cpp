#include "OximeterModule.hpp"

int32_t main(int32_t argc, char **argv) {
    OximeterModule sensor(argc, argv);

    ros::init(argc, argv, "oximeter");

    sensor.setUp();
    sensor.run();

    return 0;
}