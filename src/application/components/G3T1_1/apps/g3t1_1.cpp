#include "G3T1_1.hpp"

int32_t main(int32_t argc, char **argv) {
    G3T1_1 g3t1_1(argc, argv);

    ros::init(argc, argv, "oximeter");

    g3t1_1.setUp();
    g3t1_1.run();

    return 0;
}