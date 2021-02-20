#include "component/g3t1_4/G3T1_4.hpp"

int32_t main(int32_t argc, char **argv) {
    G3T1_4 g3t1_4(argc, argv, "abps");
    return g3t1_4.run();
}