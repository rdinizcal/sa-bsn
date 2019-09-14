#include "data_access/DataAccess.hpp"

DataAccess::DataAccess(int  &argc, char **argv, const std::string &name) : ROSComponent(argc, argv, name), fp(), event_filepath(), status_filepath(), logical_clock(0), statusVec(), eventVec() {}
DataAccess::~DataAccess() {}

int64_t DataAccess::now() const{
    return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

void DataAccess::setUp() {
    std::string path = ros::package::getPath("repository");
    event_filepath = path + "/../resource/logs/event_" + std::to_string(this->now()) + ".log";
    status_filepath = path + "/../resource/logs/status_" + std::to_string(this->now()) + ".log";
    uncertainty_filepath = path + "/../resource/logs/uncertainty_" + std::to_string(this->now()) + ".log";

    fp.open(event_filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "\n";
    fp.close();

    fp.open(status_filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "\n";
    fp.close();

    fp.open(uncertainty_filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "\n";
    fp.close();

    double freq;
	handle.getParam("frequency", freq);
	rosComponentDescriptor.setFreq(freq);
}

void DataAccess::tearDown(){}

void DataAccess::body(){
    ros::NodeHandle n;
    ros::Subscriber handle_persist = n.subscribe("persist", 1000, &DataAccess::receivePersistMessage, this);
    ros::spin();
}

void DataAccess::receivePersistMessage(const archlib::Persist::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->type.c_str());
    ++logical_clock;

    if(msg->type=="Status"){
        persistStatus(msg->timestamp, msg->source, msg->target, msg->content);
    } else if(msg->type=="Event") {
        persistEvent(msg->timestamp, msg->source, msg->target, msg->content);
    } else if(msg->type=="Uncertainty") {
        persistUncertainty(msg->timestamp, msg->source, msg->target, msg->content);
    } else {
        std::cout << "(Could not identify message type!!)" << std::endl;
    }
}

void DataAccess::persistEvent(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content){

    EventMessage obj("Event", timestamp, logical_clock, source, target, content);
    eventVec.push_back(obj);

    if(logical_clock%30==0) flush();
}

void DataAccess::persistStatus(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content){
    
    StatusMessage obj("Status", timestamp, logical_clock, source, target, content);
    statusVec.push_back(obj);

    if(logical_clock%30==0) flush();
}

void DataAccess::persistUncertainty(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content){
    
    UncertaintyMessage obj("Uncertainty", timestamp, logical_clock, source, target, content);
    uncertainVec.push_back(obj);

    if(logical_clock%30==0) flush();
}

void DataAccess::flush(){

    fp.open(status_filepath, std::fstream::in | std::fstream::out | std::fstream::app);   
    for(std::vector<StatusMessage>::iterator it = statusVec.begin(); it != statusVec.end(); ++it) {
        fp << (*it).getName() << ",";
        fp << (*it).getLogicalClock() << ",";
        fp << (*it).getTimestamp() << ",";
        fp << (*it).getSource() << ",";
        fp << (*it).getTarget() << ",";
        fp << (*it).getState() << "\n";
    }
    fp.close();
    statusVec.clear();

    fp.open(event_filepath, std::fstream::in | std::fstream::out | std::fstream::app);   
    for(std::vector<EventMessage>::iterator it = eventVec.begin(); it != eventVec.end(); ++it) {
        fp << (*it).getName() << ",";
        fp << (*it).getLogicalClock() << ",";
        fp << (*it).getTimestamp() << ",";
        fp << (*it).getSource() << ",";
        fp << (*it).getTarget() << ",";
        fp << (*it).getEvent() << "\n";
    }
    fp.close();
    eventVec.clear();

    fp.open(uncertainty_filepath, std::fstream::in | std::fstream::out | std::fstream::app);   
    for(std::vector<UncertaintyMessage>::iterator it = uncertainVec.begin(); it != uncertainVec.end(); ++it) {
        fp << (*it).getName() << ",";
        fp << (*it).getLogicalClock() << ",";
        fp << (*it).getTimestamp() << ",";
        fp << (*it).getSource() << ",";
        fp << (*it).getTarget() << ",";
        fp << (*it).getContent() << "\n";
    }
    fp.close();
    uncertainVec.clear();
}
