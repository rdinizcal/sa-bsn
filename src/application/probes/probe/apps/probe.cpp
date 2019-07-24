#include "Probe.hpp"

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "probe");
    Probe probe(argc, argv, "probe");

    probe.setUp();
    probe.run();

    return 0;
}
