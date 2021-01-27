#include "archlib/target_system/Probe.hpp"

namespace arch {
	namespace target_system {
		Probe::Probe(int &argc, char **argv, const std::string &name) : ROSComponent(argc, argv, name) {}

		Probe::~Probe() {}

        void Probe::setUp() {
            log_event = handle.advertise<archlib::Event>("log_event", 1000);
            log_status = handle.advertise<archlib::Status>("log_status", 1000);
            log_energy_status = handle.advertise<archlib::EnergyStatus>("log_energy_status", 1000);

            double freq;
	        handle.getParam("frequency", freq);
	        rosComponentDescriptor.setFreq(freq);
        }

        void Probe::tearDown() {
        }

        void Probe::collectEvent(const archlib::Event::ConstPtr& msg) {
            //ROS_INFO("I heard: [%s: %s]", msg->source.c_str(), msg->content.c_str());
            log_event.publish(msg);
        }

        void Probe::collectStatus(const archlib::Status::ConstPtr& msg) {
            //ROS_INFO("I heard: [%s: %s]", msg->source.c_str(), msg->content.c_str());
            log_status.publish(msg);
        }

        void Probe::collectEnergyStatus(const archlib::EnergyStatus::ConstPtr& msg) {
            log_energy_status.publish(msg);
        }

        void Probe::body(){
            ros::NodeHandle n;
            ros::Subscriber collect_event = n.subscribe("collect_event", 1000, &Probe::collectEvent, this);
            ros::Subscriber collect_status = n.subscribe("collect_status", 1000, &Probe::collectStatus, this);
            ros::Subscriber collect_energy_status = n.subscribe("collect_energy_status", 100, &Probe::collectEnergyStatus, this);
            ros::spin();
        }
	}
}