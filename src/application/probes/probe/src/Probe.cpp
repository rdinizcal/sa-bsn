#include "probe/Probe.hpp"

Probe::Probe(int  &argc, char **argv) {}
Probe::~Probe() {}

void Probe::setUp() {
    ros::NodeHandle handler;
    status_pub = handler.advertise<messages::Status>("log_status", 1000);
    event_pub = handler.advertise<messages::Event>("log_event", 1000);
    info_pub = handler.advertise<messages::Info>("log_info", 1000);
}

void Probe::receiveStatus(const messages::Status::ConstPtr& msg) {
    status_pub.publish(msg);
}

void Probe::receiveEvent(const messages::Event::ConstPtr& msg) {
    event_pub.publish(msg);
}

void Probe::receiveInfo(const messages::Info::ConstPtr& msg) {
    info_pub.publish(msg);
}

void Probe::run(){
    ros::NodeHandle n;
    ros::Subscriber status_sub = n.subscribe("collect_status", 1000, &Probe::receiveStatus, this);
    ros::Subscriber event_sub = n.subscribe("collect_event", 1000, &Probe::receiveEvent, this);
    ros::Subscriber info_sub = n.subscribe("collect_info", 1000, &Probe::receiveInfo, this);    
    ros::spin();
}
