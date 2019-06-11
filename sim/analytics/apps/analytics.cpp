#include "AnalyticsNode.hpp"

int main(int argc, char **argv) {
    
    AnalyticsNode analyticsNode(argc, argv, "analytics");

    ros::init(argc, argv, "analytics");

    analyticsNode.setUp();
    analyticsNode.run();

    return 0;
}