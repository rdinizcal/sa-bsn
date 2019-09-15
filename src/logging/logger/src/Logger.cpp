#include "Logger.hpp"

Logger::Logger(int  &argc, char **argv, std::string name) : fp(), filepath(), logical_clock(0), time_ref() {}
Logger::~Logger() {}

int64_t Logger::now() const{
    return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

void Logger::setUp() {
    std::string path = ros::package::getPath("logger");
    filepath = path + "/resource/logs/" + std::to_string(this->now()) + ".log";

    std::cout << filepath;

    fp.open(filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "\n";
    fp.close();

    time_ref = this->now();

    ros::NodeHandle handler;
    reconfig_logger2effector_pub = handler.advertise<messages::ReconfigurationCommand>("reconfigure", 10);
    status_logger2manager_pub = handler.advertise<messages::Status>("status", 10);
    event_logger2manager_pub = handler.advertise<messages::Event>("event", 10);
    info_logger2repository_pub = handler.advertise<messages::Info>("persist", 10);


}

void Logger::receiveReconfigurationCommand(const messages::ReconfigurationCommand::ConstPtr& msg) {
    ROS_INFO("I heard: [%s: %s]", msg->action.c_str(), msg->target.c_str());
    ++logical_clock;

    // persist
    fp.open(filepath, std::fstream::in | std::fstream::out | std::fstream::app);
    fp << "ReconfigurationCommand" << ",";
    fp << logical_clock << ",";
    fp << std::to_string(this->now()-time_ref) << ",";
    fp << msg->source << ",";
    fp << msg->target << ",";
    fp << msg->action << "\n";

    fp.close();

    //forward
    reconfig_logger2effector_pub.publish(msg);
}


void Logger::receiveStatus(const messages::Status::ConstPtr& msg) {
    ROS_INFO("I heard: [%s: %f]", msg->key.c_str(), msg->value);
    ++logical_clock;

    // persist
    fp.open(filepath, std::fstream::in | std::fstream::out | std::fstream::app);
    fp << "Status" << ",";
    fp << logical_clock << ",";
    fp << std::to_string(this->now()-time_ref) << ",";
    fp << msg->source << ",";
    fp << msg->target << ",";
    fp << msg->key << ",";
    fp << msg->value << "\n";

    fp.close();

    //forward
    status_logger2manager_pub.publish(msg);
}

void Logger::receiveEvent(const messages::Event::ConstPtr& msg) {
    ROS_INFO("I heard: [%s: %s]", msg->source.c_str(), msg->type.c_str());
    ++logical_clock;

    // persist
    fp.open(filepath, std::fstream::in | std::fstream::out | std::fstream::app);
    fp << "Event" << ",";
    fp << logical_clock << ",";
    fp << std::to_string(this->now()-time_ref) << ",";
    fp << msg->source << ",";
    fp << msg->target << ",";
    fp << msg->type << ",";
    fp << msg->description << "\n";

    fp.close();

    //forward
    event_logger2manager_pub.publish(msg);
}

void Logger::receiveInfo(const messages::Info::ConstPtr& msg) {
    info_logger2repository_pub.publish(msg);
}

void Logger::run(){
    ros::NodeHandle n;

    ros::Subscriber reconfig_sub = n.subscribe("log_reconfigure", 1000, &Logger::receiveReconfigurationCommand, this);
    ros::Subscriber status_sub = n.subscribe("log_status", 1000, &Logger::receiveStatus, this);
    ros::Subscriber event_sub = n.subscribe("log_event", 1000, &Logger::receiveEvent, this);
    ros::Subscriber info_sub = n.subscribe("log_info", 1000, &Logger::receiveInfo, this);
    
    ros::spin();
}
