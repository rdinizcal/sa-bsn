#include "component/g3t1_3/G3T1_3.hpp"

int32_t main(int32_t argc, char **argv) {
    G3T1_3 g3t1_3(argc, argv, "thermometer");
    return g3t1_3.run();
}