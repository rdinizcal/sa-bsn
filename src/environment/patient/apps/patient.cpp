#include "PatientModule.hpp"

int main(int argc, char **argv) {
    PatientModule patient(argc, argv, "patient");
    return patient.run();
}