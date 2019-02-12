#include "MonitorModule.hpp"

int32_t main(int32_t argc, char **argv) {
    MonitorModule monitor(argc, argv);
    
    return monitor.runModule();
}