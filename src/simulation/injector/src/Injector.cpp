#include "Injector.hpp"


Injector::Injector(int  &argc, char **argv, const std::string &name) : ROSComponent(argc, argv, name), cycles(0), duration(), frequency(), amplitude(), noise_factor(), begin(), end(), type() {}
Injector::~Injector() {}

void Injector::setUp() {
    srand(time(NULL));

    ros::NodeHandle config;
    double freq;
    config.getParam("frequency", freq);
    rosComponentDescriptor.setFreq(freq);

    uncertainty_pub["/g3t1_1"] = handle.advertise<archlib::Uncertainty>("uncertainty_/g3t1_1", 1000);
    uncertainty_pub["/g3t1_2"] = handle.advertise<archlib::Uncertainty>("uncertainty_/g3t1_2", 1000);
    uncertainty_pub["/g3t1_3"] = handle.advertise<archlib::Uncertainty>("uncertainty_/g3t1_3", 1000);
    uncertainty_pub["/g3t1_4"] = handle.advertise<archlib::Uncertainty>("uncertainty_/g3t1_4", 1000);

    log_uncertainty = handle.advertise<archlib::Uncertainty>("log_uncertainty", 10);

    components = {"/g3t1_1"/*, "/g3t1_2", "/g3t1_3", "/g3t1_4"*/};

    type["/g3t1_1"] = "step";
    amplitude["/g3t1_1"] = 0.15;
    frequency["/g3t1_1"] = 1.0/180000000;                                                 // once every 120 secs
    duration["/g3t1_1"] = 12000000;                                                      // 20 seconds
    noise_factor["/g3t1_1"] = 0;
    begin["/g3t1_1"] = seconds_in_cycles(30);                                                           // 1st cycle
    end["/g3t1_1"] = begin["/g3t1_1"] + seconds_in_cycles(duration["/g3t1_1"]);     // 1st cycle + the correspondent number of cycles of 20 seconds

    /*
    type["/g3t1_2"] = "ramp";
    amplitude["/g3t1_2"] = 1.0;
    frequency["/g3t1_2"] = 1.0/120; // once every 120 secs
    duration["/g3t1_2"] = 60;
    noise_factor["/g3t1_2"] = 0;
    begin["/g3t1_2"] = 10;
    end["/g3t1_2"] = begin["/g3t1_2"] + seconds_in_cycles(duration["/g3t1_2"]);

    type["/g3t1_3"] = "ramp";
    amplitude["/g3t1_3"] = 1.0;
    frequency["/g3t1_3"] = 1.0/360; // once every 120 sec
    duration["/g3t1_3"] = 180;
    noise_factor["/g3t1_3"] = 0;
    begin["/g3t1_3"] = 20;
    end["/g3t1_3"] = begin["/g3t1_3"] + seconds_in_cycles(duration["/g3t1_3"]);
    
    type["/g3t1_4"] = "ramp";
    amplitude["/g3t1_4"] = 1.0;
    frequency["/g3t1_4"] = 1.0/120; // once every 120 secs
    duration["/g3t1_4"] = 60;
    noise_factor["/g3t1_4"] = 0;
    begin["/g3t1_4"] = 30;
    end["/g3t1_4"] = begin["/g3t1_4"] + seconds_in_cycles(duration["/g3t1_4"]);
    */

}

void Injector::tearDown() {}

int64_t Injector::seconds_in_cycles(const double &seconds){
    return seconds*rosComponentDescriptor.getFreq();
}

int64_t Injector::cycles_in_seconds(const double &cycles) {
    return cycles/rosComponentDescriptor.getFreq();
}

double Injector::gen_noise(const std::string &component, double &noise, int &duration, double &amplitude, std::string &type){

    bool is_last_cycle = (cycles == end[component])?true:false;

    if(type=="step" && !is_last_cycle){
        return amplitude;
    } else if(type=="ramp" && !is_last_cycle) {
        return noise + amplitude/seconds_in_cycles(duration);
    } else if(type=="random") {
        return ((double) rand() / (RAND_MAX)) * amplitude;
    } 
    
    return 0.0;
}

void Injector::body() {
    ++cycles;

    for(std::vector<std::string>::iterator component = components.begin(); component != components.end(); ++component){
        if(begin[*component] <= cycles && cycles <= end[*component]) {

            noise_factor[*component] = gen_noise(*component, noise_factor[*component], duration[*component], amplitude[*component], type[*component]);
            inject(*component, "noise_factor=" + std::to_string(noise_factor[*component]));

            //update begin and end tags in last cycle
            if(cycles == end[*component]){
                begin[*component] += seconds_in_cycles(1.0/frequency[*component]);
                end[*component]   += seconds_in_cycles(1.0/frequency[*component]);
            }
        }
    }
}

void Injector::inject(const std::string &component, const std::string &content){
    archlib::Uncertainty msg;

    msg.source = ros::this_node::getName();
    msg.target = component;
    msg.content = content;

    uncertainty_pub[component].publish(msg);
    log_uncertainty.publish(msg);
    ROS_INFO("Inject [%s] at [%s].", content.c_str(), component.c_str());
}
