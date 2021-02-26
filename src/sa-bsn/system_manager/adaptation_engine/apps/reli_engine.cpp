#include "engine/ReliabilityEngine.hpp"

#include "ros/ros.h"

int main(int argc, char **argv) {
    ReliabilityEngine engine(argc, argv, "engine");
    return engine.run();
}