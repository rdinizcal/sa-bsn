#include "component/g4t1/G4T1.hpp"

int32_t main(int32_t argc, char **argv) {
    G4T1 g4t1(argc, argv, "centralhub");
    return g4t1.run();
}