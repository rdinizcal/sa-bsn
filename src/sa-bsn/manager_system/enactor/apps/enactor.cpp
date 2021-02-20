#include "enactor/Enactor.hpp"

int main(int argc, char **argv) {
    Enactor e(argc, argv, "enactor");
    return e.run();
}