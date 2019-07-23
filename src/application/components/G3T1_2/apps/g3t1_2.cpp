#include "G3T1_2.hpp"

int32_t main(int32_t argc, char **argv) {
    G3T1_2 g3t1_2(argc, argv);

    ros::init(argc, argv, "ecg");

    g3t1_2.setUp();
    g3t1_2.run();

    return 0;
}