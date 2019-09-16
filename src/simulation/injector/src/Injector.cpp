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
   
   log_uncertainty = handle.advertise<archlib::Uncertainty>("log_uncertainty", 10);

}

void Injector::tearDown() {}

int Injector::seconds_in_cycles(const int32_t &seconds){
    return seconds*rosComponentDescriptor.getFreq();
}

int Injector::cycles_in_seconds(const int32_t &cycles) {
    return cycles/rosComponentDescriptor.getFreq();
}

// converts the number of seconds (input) to number of cycles
bool Injector::passed_in_seconds(const int32_t &seconds) {
    if(seconds == 0) return false;
    return cycles % seconds_in_cycles(seconds) == 0;
}

void Injector::step(const std::string &component, double &amplitude) {
    injectUncertainty(component, "noise_factor=" + std::to_string(amplitude));
}

void Injector::body() {
    ++cycles;

    /* Inject step input with duration of 20 seconds at every minute, amplitude = 1*/
    if(passed_in_seconds(60)) {
        noise_factor["/g3t1_1"] = 1;
        step("/g3t1_1", noise_factor["/g3t1_1"]);
        step_duration["/g3t1_1"] = cycles_in_seconds(cycles)+20;
    }

    if(passed_in_seconds(step_duration["/g3t1_1"])) {
        noise_factor["/g3t1_1"] = 0;
        step("/g3t1_1", noise_factor["/g3t1_1"]);
        step_duration["/g3t1_1"] = 0;
    }

    /* Inject step input with duration of 10 seconds at every 30 seconds, amplitude = 1*/
    if(passed_in_seconds(30)) {
        noise_factor["/g3t1_2"] = 1;
        step("/g3t1_2", noise_factor["/g3t1_2"]);
        step_duration["/g3t1_2"] = cycles_in_seconds(cycles)+10;
    }

    if(passed_in_seconds(step_duration["/g3t1_2"])) {
        noise_factor["/g3t1_2"] = 0;
        step("/g3t1_2", noise_factor["/g3t1_2"]);
        step_duration["/g3t1_2"] = 0;
    }

    /* Inject step input with duration of 10 seconds at every 15 seconds, amplitude = 1*/
    if(passed_in_seconds(15)) {
        noise_factor["/g3t1_3"] = 1;
        step("/g3t1_3", noise_factor["/g3t1_3"]);
        step_duration["/g3t1_3"] = cycles_in_seconds(cycles)+5;
    }

    if(passed_in_seconds(step_duration["/g3t1_3"])) {
        noise_factor["/g3t1_3"] = 0;
        step("/g3t1_3", noise_factor["/g3t1_3"]);
        step_duration["/g3t1_3"] = 0;
    }
    
    /* Inject step input with duration of 10 seconds at every 20 seconds, amplitude = 1*/
    if(passed_in_seconds(20)) {
        noise_factor["/g3t1_4"] = 1;
        step("/g3t1_4", noise_factor["/g3t1_4"]);
        step_duration["/g3t1_4"] = cycles_in_seconds(cycles)+10;
    }

    if(passed_in_seconds(step_duration["/g3t1_4"])) {
        noise_factor["/g3t1_4"] = 0;
        step("/g3t1_4", noise_factor["/g3t1_4"]);
        step_duration["/g3t1_4"] = 0;
    }
}

void Injector::injectUncertainty(const std::string &component, const std::string &content){
    archlib::Uncertainty msg;

    msg.source = ros::this_node::getName();
    msg.target = component;
    msg.content = content;

    uncertainty_pub[component].publish(msg);
    log_uncertainty.publish(msg);
    ROS_INFO("Inject [%s] at [%s]!", content.c_str(), component.c_str());
}
