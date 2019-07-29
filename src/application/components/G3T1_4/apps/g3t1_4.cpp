#include "G3T1_4.hpp"

int32_t main(int32_t argc, char **argv) {
    ros::init(argc, argv, "bloodpressure");

    G3T1_4 g3t1_4(argc, argv);
    g3t1_4.run();

    return 0;
}