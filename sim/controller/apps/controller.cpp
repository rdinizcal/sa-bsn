#include "ControllerModule.hpp"

int32_t main(int32_t argc, char **argv) {
    ControllerModule cm(argc, argv);
    
    return cm.runModule();
}