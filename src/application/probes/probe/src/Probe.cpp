#include "Probe.hpp"

Probe::Probe(int  &argc, char **argv, std::string name) : task_pub() {}
Probe::~Probe() {}

void Probe::setUp() {
    ros::NodeHandle handler;
    task_pub = handler.advertise<messages::TaskInfo>("probe_logger", 10);
}

void Probe::receiveTaskInfo(const messages::TaskInfo::ConstPtr& msg) {
    //ROS_INFO("I heard: [%s]", msg->task_id.c_str());
    task_pub.publish(msg);
}

void Probe::run(){
    ros::NodeHandle n;
    ros::Subscriber task_sub = n.subscribe("task_info", 1000, &Probe::receiveTaskInfo, this);
    ros::spin();
}
