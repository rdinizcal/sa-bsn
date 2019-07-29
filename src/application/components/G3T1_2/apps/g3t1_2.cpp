#include "G3T1_2.hpp"

int32_t main(int32_t argc, char **argv) {
    ros::init(argc, argv, "ecg");

    G3T1_2 g3t1_2(argc, argv);
    g3t1_2.run();

    return 0;
}