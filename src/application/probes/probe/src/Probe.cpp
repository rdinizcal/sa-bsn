#include "Probe.hpp"

Probe::Probe(int  &argc, char **argv, std::string name) : task_pub(), context_pub() {}
Probe::~Probe() {}

void Probe::setUp() {
    ros::NodeHandle handler;
    task_pub = handler.advertise<messages::TaskInfo>("probe_logger", 10);
    context_pub = handler.advertise<messages::ContextInfo>("probe_logger", 10);

}

void Probe::receiveTaskInfo(const messages::TaskInfo::ConstPtr& msg) {
    task_pub.publish(msg);
}

void Probe::receiveContextInfo(const messages::ContextInfo::ConstPtr& msg) {
    context_pub.publish(msg);
}

void Probe::run(){
    ros::NodeHandle n;
    ros::Subscriber task_sub = n.subscribe("task_info", 1000, &Probe::receiveTaskInfo, this);
    ros::Subscriber context_sub = n.subscribe("context_info", 1000, &Probe::receiveContextInfo, this);
    ros::spin();
}
