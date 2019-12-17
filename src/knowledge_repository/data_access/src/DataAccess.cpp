#include "data_access/DataAccess.hpp"

DataAccess::DataAccess(int  &argc, char **argv, const std::string &name) : ROSComponent(argc, argv, name), fp(), event_filepath(), status_filepath(), logical_clock(0), statusVec(), eventVec(), status(), buffer_size() {}
DataAccess::~DataAccess() {}

int64_t DataAccess::now() const{
    return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

void DataAccess::setUp() {
    std::string path = ros::package::getPath("repository");

    file_id = std::to_string(this->now());

    event_filepath = path + "/../resource/logs/event_" + file_id + ".log";
    tmp_event_filepath = path + "/../resource/logs/event_" + file_id + "_tmp.log";
    status_filepath = path + "/../resource/logs/status_" + file_id + ".log";
    tmp_status_filepath = path + "/../resource/logs/status_" + file_id + "_tmp.log";
    uncertainty_filepath = path + "/../resource/logs/uncertainty_" + file_id + ".log";
    adaptation_filepath = path + "/../resource/logs/adaptation_" + file_id + ".log";
    //ctmetrics_filepath = path + "/../resource/logs/ctmetrics_" + file_id + ".log";
    //engineinfo_filepath = path + "/../resource/logs/engineinfo_" + file_id + ".log";
    ctmetrics_filepath = path + "/../resource/logs/ctmetrics.log";
    engineinfo_filepath = path + "/../resource/logs/engineinfo.log";

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
}

void DataAccess::tearDown(){}

void DataAccess::body(){
    ros::NodeHandle n;
    ros::Subscriber handle_persist = n.subscribe("persist", 1000, &DataAccess::receivePersistMessage, this);
    ros::ServiceServer server = handle.advertiseService("DataAccessRequest", &DataAccess::processQuery, this);
    ros::ServiceServer adr_server = handle.advertiseService("address", &DataAccess::sendAddress, this);
    ros::spin();
}

void DataAccess::receivePersistMessage(const archlib::Persist::ConstPtr& msg) {
    //ROS_INFO("I heard: [%s]", msg->type.c_str());
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
    } else if(msg->type=="ControlTheoryMetrics") {
        persistControlTheoryMetrics(msg->timestamp, msg->source, msg->target, msg->content);
    } else if(msg->type=="EngineInfo") {
        persistEngineInfo(msg->timestamp, msg->source, msg->target, msg->content);
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
                    for(int i = it->second.size()-num; i < it->second.size(); ++i){
                        flag = true;
                        aux += it->second[i];
                        //if((i < num) && (i+1 < it->second.size())) 
                        aux += ",";
                    }
                    aux += ";";

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

bool DataAccess::sendAddress(services::Address::Request &req, services::Address::Response &res){
    ROS_INFO("Received address request");
    res.id = file_id;
    return true;
}

void DataAccess::persistEvent(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content){

    EventMessage obj("Event", timestamp, logical_clock, source, target, content);
    eventVec.push_back(obj);
    eventVecTmp.push_back(obj);

    if(logical_clock%30==0) flush();
}

void DataAccess::persistStatus(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content){
    
    StatusMessage obj("Status", timestamp, logical_clock, source, target, content);
    statusVec.push_back(obj);
    statusVecTmp.push_back(obj);

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

void DataAccess::persistControlTheoryMetrics(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content){
    
    //parsing
    bsn::operation::Operation op;
    std::vector<std::string> metrics = op.split(content, ';');

    ControlTheoryMetricsMessage obj("ControlTheoryMetrics", timestamp, logical_clock, source, target, metrics[0], metrics[1], metrics[2], metrics[3], metrics[4], metrics[5], metrics[6]);
    ctmetricsVec.push_back(obj);

    if(logical_clock%30==0) flush();
}

void DataAccess::persistEngineInfo(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content) {
    bsn::operation::Operation op;
    std::vector<std::string> data = op.split(content, ';');

    EngineInfoMessage obj("EngineInfo", timestamp, logical_clock, source, target, data[0], data[1], data[2]);
    engineinfoVec.push_back(obj);

    if(logical_clock%30==0) flush();
}

void DataAccess::flush(){
    fp.open(engineinfo_filepath, std::fstream::in | std::fstream::out | std::fstream::app);
    for(std::vector<EngineInfoMessage>::iterator it = engineinfoVec.begin(); it != engineinfoVec.end(); ++it)
    {
        fp << (*it).getName() << ",";
        fp << (*it).getLogicalClock() << ",";
        fp << (*it).getTimestamp() << ",";
        fp << (*it).getSource() << ",";
        fp << (*it).getTarget() << ",";
        fp << (*it).getEngineKp() << ",";
        fp << (*it).getEngineOffset() << ",";
        fp << (*it).getElapsedTime() << "\n";
        
    }
    fp.close();
    engineinfoVec.clear();

    fp.open(ctmetrics_filepath, std::fstream::in | std::fstream::out | std::fstream::app);
    for(std::vector<ControlTheoryMetricsMessage>::iterator it = ctmetricsVec.begin(); it != ctmetricsVec.end(); ++it)
    {
        fp << (*it).getName() << ",";
        fp << (*it).getLogicalClock() << ",";
        fp << (*it).getTimestamp() << ",";
        fp << (*it).getSource() << ",";
        fp << (*it).getTarget() << ",";
        fp << (*it).getEnactorKp() << ",";
        fp << (*it).getEnactorKi() << ",";
        fp << (*it).getStability() << ",";
        fp << (*it).getConvergencePoint() << ",";
        fp << (*it).getSettlingTime() << ",";
        fp << (*it).getOvershoot() << ",";
        fp << (*it).getSteadyStateError() << "\n";
    }
    fp.close();
    ctmetricsVec.clear();

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

    fp.open(tmp_status_filepath, std::fstream::in | std::fstream::out | std::fstream::app);   
    for(std::vector<StatusMessage>::iterator it = statusVecTmp.begin(); it != statusVecTmp.end(); ++it) {
        fp << (*it).getName() << ",";
        fp << (*it).getLogicalClock() << ",";
        fp << (*it).getTimestamp() << ",";
        fp << (*it).getSource() << ",";
        fp << (*it).getTarget() << ",";
        fp << (*it).getState() << "\n";
    }
    fp.close();
    statusVecTmp.clear();

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

    fp.open(tmp_event_filepath, std::fstream::in | std::fstream::out | std::fstream::app);   
    for(std::vector<EventMessage>::iterator it = eventVecTmp.begin(); it != eventVecTmp.end(); ++it) {
        fp << (*it).getName() << ",";
        fp << (*it).getLogicalClock() << ",";
        fp << (*it).getTimestamp() << ",";
        fp << (*it).getSource() << ",";
        fp << (*it).getTarget() << ",";
        fp << (*it).getEvent() << "\n";
    }
    fp.close();
    eventVecTmp.clear();

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
