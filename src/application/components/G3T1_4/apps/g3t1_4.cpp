#include "G3T1_4.hpp"

int32_t main(int32_t argc, char **argv) {
    G3T1_4 g3t1_4(argc, argv);

    ros::init(argc, argv, "bloodpressure");

    g3t1_4.setUp();
    g3t1_4.run();

    return 0;
}