#include "component/g3t1_6/G3T1_6.hpp"

int32_t main(int32_t argc, char **argv) {
    G3T1_6 g3t1_6(argc, argv, "glucosemeter");
    return g3t1_6.run();
}
