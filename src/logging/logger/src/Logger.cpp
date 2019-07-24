#include "Logger.hpp"

Logger::Logger(int  &argc, char **argv, std::string name) : fp(), filepath(), logical_clock(0) {}

Logger::~Logger() {}

std::string Logger::now() const{
    return std::to_string(std::chrono::high_resolution_clock::now().time_since_epoch().count());
}

void Logger::setUp() {
    std::string path = ros::package::getPath("logger");
    std::string now = this->now();

    filepath = path + "/resource/logs/" + now + ".log";
    std::cout << filepath;

    fp.open(filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "\n";
    fp.close();
}

void Logger::receiveTaskInfo(const messages::TaskInfo::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->task_id.c_str());
    ++logical_clock;

    // persist
    fp.open(filepath, std::fstream::in | std::fstream::out | std::fstream::app);
    fp << logical_clock << ",";
    fp << this->now() << ",";
    fp << msg->task_id << ",";
    fp << msg->cost << ",";
    fp << msg->reliability << ",";
    fp << msg->frequency << "\n";

    fp.close();

    //forward

}

// Analytics receives information from both tasks and contexts of sensors

void Logger::run(){

    ros::NodeHandle n;
    ros::Subscriber t_sub = n.subscribe("probe_logger", 1000, &Logger::receiveTaskInfo, this);
    ros::spin();

}
