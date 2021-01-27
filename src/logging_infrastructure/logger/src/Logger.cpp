#include "Logger.hpp"


Logger::Logger(int  &argc, char **argv, const std::string &name) : ROSComponent(argc, argv, name), time_ref() {}
Logger::~Logger() {}

int64_t Logger::now() const{
    return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

void Logger::setUp() {
    time_ref = this->now();

    adapt = handle.advertise<archlib::AdaptationCommand>("reconfigure", 1000);
    persist = handle.advertise<archlib::Persist>("persist", 1000);
    status = handle.advertise<archlib::Status>("status", 1000);
    event = handle.advertise<archlib::Event>("event", 1000);

    double freq;
	handle.getParam("frequency", freq);
	rosComponentDescriptor.setFreq(freq);
}

void Logger::tearDown() {}

void Logger::body() {
    ros::NodeHandle n;
    ros::Subscriber reconfig_sub = n.subscribe("log_adapt", 1000, &Logger::receiveAdaptationCommand, this);
    ros::Subscriber status_sub = n.subscribe("log_status", 1000, &Logger::receiveStatus, this);
    ros::Subscriber energy_status_sub = n.subscribe("log_energy_status", 1000, &Logger::receiveEnergyStatus, this);
    ros::Subscriber event_sub = n.subscribe("log_event", 1000, &Logger::receiveEvent, this);
    ros::Subscriber uncertainty_sub = n.subscribe("log_uncertainty", 1000, &Logger::receiveUncertainty, this);
    ros::spin();
}

void Logger::receiveAdaptationCommand(const archlib::AdaptationCommand::ConstPtr& msg) {
    //ROS_INFO("I heard: [%s: %s]", msg->action.c_str(), msg->target.c_str());

    archlib::Persist persistMsg;
    persistMsg.source = msg->source;
    persistMsg.target = msg->target;
    persistMsg.type = "AdaptationCommand";
    persistMsg.timestamp = this->now()-time_ref;
    persistMsg.content = msg->action;

    persist.publish(persistMsg);
    adapt.publish(msg);
}


void Logger::receiveStatus(const archlib::Status::ConstPtr& msg) {
    //ROS_INFO("I heard: [%s: %s]", msg->source.c_str(), msg->content.c_str());

    archlib::Persist persistMsg;
    persistMsg.source = msg->source;
    persistMsg.target = msg->target;
    persistMsg.type = "Status";
    persistMsg.timestamp = this->now()-time_ref;
    persistMsg.content = msg->content;

    persist.publish(persistMsg);
    status.publish(msg);
}

void Logger::receiveEnergyStatus(const archlib::EnergyStatus::ConstPtr& msg) {
    //ROS_INFO("I heard: [%s: %s]", msg->source.c_str(), to_string(msg->content).c_str());

    archlib::Persist persistMsg;
    persistMsg.source = msg->source;
    persistMsg.target = msg->target;
    persistMsg.type = "EnergyStatus";
    persistMsg.timestamp = this->now()-time_ref;
    persistMsg.content = msg->content;

    persist.publish(persistMsg);
}

void Logger::receiveEvent(const archlib::Event::ConstPtr& msg) {
    //ROS_INFO("I heard: [%s: %s]", msg->source.c_str(), msg->content.c_str());

    archlib::Persist persistMsg;
    persistMsg.source = msg->source;
    persistMsg.target = msg->target;
    persistMsg.type = "Event";
    persistMsg.timestamp = this->now()-time_ref;
    persistMsg.content = msg->content;

    persist.publish(persistMsg);
    event.publish(msg);
}

void Logger::receiveUncertainty(const archlib::Uncertainty::ConstPtr& msg) {
    //ROS_INFO("I heard: [%s: %s]", msg->source.c_str(), msg->content.c_str());

    archlib::Persist persistMsg;
    persistMsg.source = msg->source;
    persistMsg.target = msg->target;
    persistMsg.type = "Uncertainty";
    persistMsg.timestamp = this->now()-time_ref;
    persistMsg.content = msg->content;

    persist.publish(persistMsg);
}