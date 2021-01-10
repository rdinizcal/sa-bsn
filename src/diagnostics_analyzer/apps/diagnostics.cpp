#include "PropertyAnalyzer.hpp"

int main(int argc, char **argv) {
    PropertyAnalyzer diagnostics(argc, argv, "diagnostics analyzer");
    diagnostics.run();
}