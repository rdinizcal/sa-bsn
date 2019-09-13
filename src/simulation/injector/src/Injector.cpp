#include "Injector.hpp"


Injector::Injector(int  &argc, char **argv, const std::string &name) : ROSComponent(argc, argv, name), cycles(0) {}
Injector::~Injector() {}

void Injector::setUp() {
    ros::NodeHandle config;
    double freq;
	config.getParam("frequency", freq);
	rosComponentDescriptor.setFreq(freq);

   uncertainty_pub["/g3t1_1"] = handle.advertise<archlib::Uncertainty>("uncertainty_/g3t1_1", 10);
   uncertainty_pub["/g3t1_2"] = handle.advertise<archlib::Uncertainty>("uncertainty_/g3t1_2", 10);
   uncertainty_pub["/g3t1_3"] = handle.advertise<archlib::Uncertainty>("uncertainty_/g3t1_3", 10);
   uncertainty_pub["/g3t1_4"] = handle.advertise<archlib::Uncertainty>("uncertainty_/g3t1_4", 10);
}

void Injector::tearDown() {}

// converts the number of seconds (input) to number of cycles
int Injector::second_to_cycles(const int32_t &seconds) {
    return seconds*rosComponentDescriptor.getFreq();
}

void Injector::body() {
    ++cycles;

    if(cycles % second_to_cycles(60) == 0 && noise_factor["/g3t1_1"] < 1) {
        noise_factor["/g3t1_1"] += 0.2;
        injectUncertainty("/g3t1_1", "noise_factor=" + std::to_string(noise_factor["/g3t1_1"]));
    }

    if(cycles % second_to_cycles(30) == 0 && noise_factor["/g3t1_2"] < 1) {
        noise_factor["/g3t1_2"] += 0.1;
        injectUncertainty("/g3t1_2", "noise_factor=" + std::to_string(noise_factor["/g3t1_2"]));
    }

    if(cycles % second_to_cycles(15) == 0 && noise_factor["/g3t1_3"] < 1) {
        noise_factor["/g3t1_3"] += 0.03;
        injectUncertainty("/g3t1_3", "noise_factor=" + std::to_string(noise_factor["/g3t1_3"]));
    }

    if(cycles % second_to_cycles(5) == 0 && noise_factor["/g3t1_4"] < 1) {
        noise_factor["/g3t1_4"] += 0.02;
        injectUncertainty("/g3t1_4", "noise_factor=" + std::to_string(noise_factor["/g3t1_4"]));
    }
}

void Injector::injectUncertainty(const std::string &component, const std::string &content){
    archlib::Uncertainty msg;

    msg.source = ros::this_node::getName();
    msg.target = component;
    msg.content = content;

    uncertainty_pub[component].publish(msg);
    ROS_INFO("Inject [%s] at [%s]!", content.c_str(), component.c_str());
}
