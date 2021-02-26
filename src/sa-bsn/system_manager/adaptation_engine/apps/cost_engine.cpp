#include "engine/CostEngine.hpp"

#include "ros/ros.h"

int main(int argc, char **argv) {
    CostEngine engine(argc, argv, "engine");
    return engine.run();
}