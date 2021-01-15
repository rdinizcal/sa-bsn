#include "DiagnosticsLogger.hpp"

int main(int argc, char **argv) {
    DiagnosticsLogger diagnostics(argc, argv, "diagnostics logger");
    diagnostics.run();
}