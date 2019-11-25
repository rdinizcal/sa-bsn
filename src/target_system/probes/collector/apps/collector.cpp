#include "archlib/target_system/Probe.hpp"

int main(int argc, char **argv) {
    arch::target_system::Probe probe(argc, argv, "collector");
    return probe.run();
}
