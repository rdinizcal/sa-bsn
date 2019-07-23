#include "CentralhubModule.hpp"

int32_t main(int32_t argc, char **argv) {
    CentralhubModule centralhub(argc, argv);

    ros::init(argc, argv, "centrahub");

    centralhub.setUp();
    centralhub.run();

    return 0;
}