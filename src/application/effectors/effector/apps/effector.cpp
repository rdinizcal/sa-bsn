#include "effector/Effector.hpp"

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "effector");
    Effector effector(argc, argv);

    effector.setUp();
    effector.run();

    return 0;
}
