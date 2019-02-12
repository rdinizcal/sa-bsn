#include "CentralhubModule.hpp"

int32_t main(int32_t argc, char **argv) {
    CentralhubModule centralhub(argc, argv);

    return centralhub.runModule();
}