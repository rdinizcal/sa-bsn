#include "ControllerNode.hpp"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
    
    ControllerNode controllerNode(argc, argv, "controller");

    ros::init(argc, argv, "controller");

    controllerNode.setUp();
    controllerNode.run();

    return 0;
}