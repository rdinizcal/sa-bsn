#include "paramadapter/ParamAdapter.hpp"

int32_t main(int argc, char **argv) {
    ParamAdapter paramadapter(argc, argv, "paramadapter");

    return paramadapter.run();
}
