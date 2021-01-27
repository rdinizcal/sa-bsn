#include "controller/Controller.hpp"

int main(int argc, char **argv) {
    Controller c(argc, argv, "enactor");
    return c.run();
}