#include "DiagnosticsAnalyzer.hpp"

int main(int argc, char **argv) {
    DiagnosticsAnalyzer diagnostics(argc, argv, "diagnostics analyzer");
    diagnostics.run();
}