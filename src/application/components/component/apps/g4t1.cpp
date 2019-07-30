#include "component/g4t1/G4T1.hpp"

int32_t main(int32_t argc, char **argv) {
    ros::init(argc, argv, "centrahub");

    G4T1 g4t1(argc, argv);
    g4t1.run();

    return 0;
}