#include "Probe.hpp"

Probe::Probe(int  &argc, char **argv, std::string name) {}
Probe::~Probe() {}

void Probe::setUp() {
    ros::NodeHandle handler;
    status_pub = handler.advertise<messages::Status>("log_status", 10);
}

void Probe::receiveStatus(const messages::Status::ConstPtr& msg) {
    status_pub.publish(msg);
}

void Probe::run(){
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("collect_status", 1000, &Probe::receiveStatus, this);
    ros::spin();
}
