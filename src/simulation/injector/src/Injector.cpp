#include "Injector.hpp"


Injector::Injector(int  &argc, char **argv, const std::string &name) : ROSComponent(argc, argv, name), cycles(0), noise_factor(0.1) {}
Injector::~Injector() {}

void Injector::setUp() {
    ros::NodeHandle config;
    double freq;
	config.getParam("frequency", freq);
	rosComponentDescriptor.setFreq(freq);

   uncertainty_g3t1_1 = handle.advertise<archlib::Uncertainty>("uncertainty_/g3t1_1", 10);
}

void Injector::tearDown() {}

// converts the number of seconds (input) to number of cycles
int Injector::second_to_cycles(const int32_t &seconds) {
    return seconds*rosComponentDescriptor.getFreq();
}

void Injector::body() {
    ++cycles;

    if(cycles % second_to_cycles(60) == 0) {
        noise_factor*=2;
        injectUncertainty("/g3t1_1", "noise_factor=" + std::to_string(noise_factor));
    }
}

void Injector::injectUncertainty(const std::string &component, const std::string &content){
    archlib::Uncertainty msg;

    msg.source = ros::this_node::getName();
    msg.target = component;
    msg.content = content;

    uncertainty_g3t1_1.publish(msg);
    ROS_INFO("Inject [%s] at [%s]!", content.c_str(), component.c_str());
}
