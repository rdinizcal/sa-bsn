#include "component/g3t1_3/G3T1_3.hpp"

int32_t main(int32_t argc, char **argv) {
    ros::init(argc, argv, "thermometer");

    G3T1_3 g3t1_3(argc, argv);
    g3t1_3.run();

    return 0;
}