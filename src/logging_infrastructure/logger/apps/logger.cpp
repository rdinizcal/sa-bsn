#include "Logger.hpp"

int32_t main(int argc, char **argv) {    
    Logger logger(argc, argv, "logger");
    return logger.run();
}
