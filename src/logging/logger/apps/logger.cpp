#include "Logger.hpp"

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "logger");
    Logger logger(argc, argv, "logger");

    logger.setUp();
    logger.run();

    return 0;
}
