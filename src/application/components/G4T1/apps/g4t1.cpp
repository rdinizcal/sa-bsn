#include "G4T1.hpp"

int32_t main(int32_t argc, char **argv) {
    G4T1 g4t1(argc, argv);

    ros::init(argc, argv, "centrahub");

    g4t1.setUp();
    g4t1.run();

    return 0;
}