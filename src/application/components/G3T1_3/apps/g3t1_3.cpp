#include "G3T1_3.hpp"

int32_t main(int32_t argc, char **argv) {
    G3T1_3 g3t1_3(argc, argv);

    ros::init(argc, argv, "thermometer");
    
    g3t1_3.setUp();
    g3t1_3.run();

    return 0;
}