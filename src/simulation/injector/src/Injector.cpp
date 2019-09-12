#include "Injector.hpp"


Injector::Injector(int  &argc, char **argv, const std::string &name) : ROSComponent(argc, argv, name), cycles(0), noise_factor(0.1) {}
Injector::~Injector() {}

void Injector::setUp() {
    ros::NodeHandle config;
    double freq;
	config.getParam("frequency", freq);
	rosComponentDescriptor.setFreq(freq);
}

void Injector::tearDown() {}

// converts the number of seconds (input) to number of cycles
void Injector::second_to_cycles(const int32_t &seconds) {
    return seconds*rosComponent.getFreq();
}

freq = 10 -> executes 10 times / second

60 seconds = 600 cycles


void Injector::body() {
    ++cycles;

    if(cycles % secs(60) == 0) {
        noise_factor*=2;
        injectUncertainty("g3t1_1", "noise_factor="+noise_factor);
    }
}

void Injector::injectUncertainty(const std::string &component, const std::string &content){
    uncertainty = handle.advertise<archlib::Uncertainty>("uncertainty_/"+component, 10);

    
}
