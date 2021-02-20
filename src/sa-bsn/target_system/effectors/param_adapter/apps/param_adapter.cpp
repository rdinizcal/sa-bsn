#include "param_adapter/ParamAdapter.hpp"

int32_t main(int argc, char **argv) {
    ParamAdapter paramadapter(argc, argv, "param_adapter");

    return paramadapter.run();
}
