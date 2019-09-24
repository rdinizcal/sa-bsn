#include "PatientModule.hpp"

int main(int argc, char **argv) {
    PatientModule patient();

    ros::init(argc, argv, "patient");
    
    patient.setUp();
    patient.run();
    s
    return 0;
}