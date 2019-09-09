#include "component/g3t1_2/G3T1_2.hpp"

int32_t main(int32_t argc, char **argv) {
    G3T1_2 g3t1_2(argc, argv, "ecg");
    return g3t1_2.run();
}