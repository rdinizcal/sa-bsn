#include "engine/Engine.hpp"

int main(int argc, char **argv) {
    Engine e(argc, argv, "engine");
    return e.run();
}