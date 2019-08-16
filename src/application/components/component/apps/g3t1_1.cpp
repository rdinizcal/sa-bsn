#include "component/g3t1_1/G3T1_1.hpp"

int32_t main(int32_t argc, char **argv) {
    ros::init(argc, argv, "oximeter");

    G3T1_1 g3t1_1(argc, argv);
    g3t1_1.run();

    return 0;
}