#include "Injector.hpp"

int32_t main(int argc, char **argv) {    
    Injector injector(argc, argv, "injector");
    return injector.run();
}
