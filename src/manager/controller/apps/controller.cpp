#include "controller/Controller.hpp"

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "controller");
    Controller controller(argc, argv, "controller");

    controller.setUp();
    controller.run();

    return 0;
}