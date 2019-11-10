#include "data_access/DataAccess.hpp"

DataAccess::DataAccess(int  &argc, char **argv, const std::string &name) : ROSComponent(argc, argv, name), fp(), event_filepath(), status_filepath(), logical_clock(0), statusVec(), eventVec(), status(), buffer_size() {}
DataAccess::~DataAccess() {}

#define W(x) std::cerr << #x << " " << x << std::endl;

int64_t DataAccess::now() const{
    return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

void DataAccess::setUp() {
    std::string path = ros::package::getPath("repository");

    std::string now = std::to_string(this->now());

    event_filepath = path + "/../resource/logs/event_" + now + ".log";
    status_filepath = path + "/../resource/logs/status_" + now + ".log";
    uncertainty_filepath = path + "/../resource/logs/uncertainty_" + now + ".log";
    adaptation_filepath = path + "/../resource/logs/adaptation_" + now + ".log";

    fp.open(event_filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "\n";
    fp.close();

    fp.open(status_filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "\n";
    fp.close();

    fp.open(uncertainty_filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "\n";
    fp.close();

    fp.open(adaptation_filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "\n";
    fp.close();

    double freq;
	handle.getParam("frequency", freq);
	rosComponentDescriptor.setFreq(freq);

    buffer_size = 1000;

    handle.getParam("send_to_srv", connected);
}

void DataAccess::tearDown(){}

void DataAccess::processTargetSystemData(const messages::TargetSystemData::ConstPtr &msg) {
    if (connected) {
        if (componentsBatteries["G3T1_1"] > msg->trm_batt) {
            componentsCosts["G3T1_1"] = componentsBatteries["G3T1_1"] - msg->trm_batt;
        }
        if (componentsBatteries["G3T1_2"] > msg->ecg_batt) {
            componentsCosts["G3T1_2"] = componentsBatteries["G3T1_2"] - msg->ecg_batt;
        }
        if (componentsBatteries["G3T1_3"] > msg->oxi_batt) {
            componentsCosts["G3T1_3"] = componentsBatteries["G3T1_3"] - msg->oxi_batt;
        }

        componentsBatteries["G3T1_1"] = msg->trm_batt;
        componentsBatteries["G3T1_2"] = msg->ecg_batt;
        componentsBatteries["G3T1_3"] = msg->oxi_batt;
        W(msg->trm_batt);
        W(msg->trm_data);
    }
}

void DataAccess::body(){
    ros::NodeHandle n;
    ros::Subscriber handle_persist = n.subscribe("persist", 1000, &DataAccess::receivePersistMessage, this);
    ros::ServiceServer server = handle.advertiseService("DataAccessRequest", &DataAccess::processQuery, this);

    ros::Subscriber targetSystemSub = n.subscribe("TargeSystemData", 100, &DataAccess::processTargetSystemData, this);
    ros::spin();
}

void DataAccess::receivePersistMessage(const archlib::Persist::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->type.c_str());
    ++logical_clock;

    if(msg->type=="Status"){
        persistStatus(msg->timestamp, msg->source, msg->target, msg->content);
        if(status[msg->source].size()<=buffer_size) {
            status[msg->source].push_back(msg->content);
        } else {
            status[msg->source].pop_front();
            status[msg->source].push_back(msg->content);
        }
    } else if(msg->type=="Event") {
        persistEvent(msg->timestamp, msg->source, msg->target, msg->content);
        if(events[msg->source].size()<=buffer_size) {
            events[msg->source].push_back(msg->content);
        } else {
            events[msg->source].pop_front();
            events[msg->source].push_back(msg->content);
        }
    } else if(msg->type=="Uncertainty") {
        persistUncertainty(msg->timestamp, msg->source, msg->target, msg->content);
    } else if(msg->type=="AdaptationCommand") {
        persistAdaptation(msg->timestamp, msg->source, msg->target, msg->content);
    } else {
        ROS_INFO("(Could not identify message type!!)");
    }
}

bool DataAccess::processQuery(archlib::DataAccessRequest::Request &req, archlib::DataAccessRequest::Response &res){

    res.content = "";

    try {

        if(req.name == "/engine") {
            bsn::operation::Operation op;
            // wait smth like "all:status:100" -> return the last 100 success and failures of all active modules
            std::vector<std::string> query = op.split(req.query,':');

            if(query[1] == "status"){
                int num = stoi(query[2]);

                for(std::map<std::string, std::deque<std::string>>::iterator it = status.begin(); it != status.end(); it++){
                    std::string aux = it->first;
                    aux += ":";
                    bool flag = false;
                    double sum = 0;
                    uint32_t len = 0;
                    for(int i = it->second.size()-num; i < it->second.size(); ++i) {
                        if (it->second[i] == "success") { // calculate reliability
                            sum +=1;
                            len++;
                        } else if (it->second[i] == "fail") {
                            len++;
                        }
                        else {
                            aux += it->second[i];
                            aux += ",";
                        }
                        
                        flag = true;
                        //if((i < num) && (i+1 < it->second.size())) 
                    }
                    aux += std::to_string((len > 0) ? sum / len : 0) + ';';
                    componentsReliabilities[it->first] = (len > 0) ? sum / len : 0;

                    if(flag) res.content += aux;
                }
            } else if (query[1] == "event") {
                int num = stoi(query[2]);

                for(std::map<std::string, std::deque<std::string>>::iterator it = events.begin(); it != events.end(); it++){
                    std::string aux = it->first;
                    aux += ":";
                    bool flag = false;
                    for(int i = it->second.size()-num; i < it->second.size(); ++i){
                        flag = true;
                        aux += it->second[i];
                        //if(i < num && i+1 < it->second.size()) 
                        aux += ",";
                    }
                    aux += ";";

                    if(flag) res.content += aux;
                }
            }
        } 

    } catch(...) {}
	

    return true;
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

void DataAccess::persistAdaptation(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content){
    
    AdaptationMessage obj("Adaptation", timestamp, logical_clock, source, target, content);
    adaptVec.push_back(obj);

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

    fp.open(adaptation_filepath, std::fstream::in | std::fstream::out | std::fstream::app);   
    for(std::vector<AdaptationMessage>::iterator it = adaptVec.begin(); it != adaptVec.end(); ++it) {
        fp << (*it).getName() << ",";
        fp << (*it).getLogicalClock() << ",";
        fp << (*it).getTimestamp() << ",";
        fp << (*it).getSource() << ",";
        fp << (*it).getTarget() << ",";
        fp << (*it).getContent() << "\n";
    }
    fp.close();
    adaptVec.clear();
}
