#include "data_access/DataAccess.hpp"

#define W(x) std::cerr << #x << " = " << x << std::endl;

DataAccess::DataAccess(int  &argc, char **argv, const std::string &name) : ROSComponent(argc, argv, name), fp(), event_filepath(), status_filepath(), logical_clock(0), statusVec(), eventVec(), status(), buffer_size(), reliability_formula(), cost_formula() {}
DataAccess::~DataAccess() {}

int64_t DataAccess::now() const{
    return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

std::chrono::high_resolution_clock::time_point DataAccess::nowInSeconds() const {
    return std::chrono::high_resolution_clock::now();
}

std::string fetch_formula(std::string name){
    std::string formula;
    std::string path = ros::package::getPath("repository");

    //std::string path = ros::package::getPath("adaptation_engine");
    std::string filename = "/../resource/models/" + name + ".formula";

    try{
        std::ifstream file;
        file.open(path + filename);
        std::getline(file, formula);
        file.close();
    } catch (std::ifstream::failure e) { 
        std::cerr << "Error: Could not load " + name +  " formula into memory\n"; 
    }

    return formula;
}

void DataAccess::setUp() {
    std::string path = ros::package::getPath("repository");
    std::string url;
    std::string now = std::to_string(this->now());

    event_filepath = path + "/../resource/logs/event_" + now + ".log";
    status_filepath = path + "/../resource/logs/status_" + now + ".log";
    energy_status_filepath = path + "/../resource/logs/energystatus_" + now + ".log";
    uncertainty_filepath = path + "/../resource/logs/uncertainty_" + now + ".log";
    adaptation_filepath = path + "/../resource/logs/adaptation_" + now + ".log";

    fp.open(event_filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "\n";
    fp.close();

    fp.open(status_filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "\n";
    fp.close();

    fp.open(energy_status_filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "\n";
    fp.close();

    fp.open(uncertainty_filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "\n";
    fp.close();

    fp.open(adaptation_filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "\n";
    fp.close();

	handle.getParam("frequency", frequency);
    rosComponentDescriptor.setFreq(frequency);

    buffer_size = 1000;

    reliability_formula = fetch_formula("reliability");
    cost_formula = fetch_formula("cost");
    
    count_to_calc_and_reset = 0;
    arrived_status = 0;

    components_batteries["g3t1_1"] = 100;
    components_batteries["g3t1_2"] = 100;
    components_batteries["g3t1_3"] = 100;
    components_batteries["g3t1_4"] = 100;
    components_batteries["g3t1_5"] = 100;
    components_batteries["g3t1_6"] = 100;
    components_costs_enactor["g3t1_1"] = 0;
    components_costs_enactor["g3t1_2"] = 0;
    components_costs_enactor["g3t1_3"] = 0;
    components_costs_enactor["g3t1_4"] = 0;
    components_costs_enactor["g3t1_5"] = 0;
    components_costs_enactor["g3t1_6"] = 0;
    components_costs_engine["g3t1_1"] = 0;
    components_costs_engine["g3t1_2"] = 0;
    components_costs_engine["g3t1_3"] = 0;
    components_costs_engine["g3t1_4"] = 0;
    components_costs_engine["g3t1_5"] = 0;
    components_costs_engine["g3t1_6"] = 0;
    components_reliabilities["g3t1_1"] = 1;
    components_reliabilities["g3t1_2"] = 1;
    components_reliabilities["g3t1_3"] = 1;
    components_reliabilities["g3t1_4"] = 1;
    components_reliabilities["g3t1_5"] = 1;
    components_reliabilities["g3t1_6"] = 1;
    
    handle_persist = handle.subscribe("persist", 1000, &DataAccess::receivePersistMessage, this);
    server = handle.advertiseService("DataAccessRequest", &DataAccess::processQuery, this);
    targetSystemSub = handle.subscribe("TargetSystemData", 100, &DataAccess::processTargetSystemData, this);
}

void DataAccess::tearDown(){}

void DataAccess::processTargetSystemData(const messages::TargetSystemData::ConstPtr& msg) {
    components_batteries["g3t1_1"] = msg->trm_batt;
    components_batteries["g3t1_2"] = msg->ecg_batt;
    components_batteries["g3t1_3"] = msg->oxi_batt;
    components_batteries["g3t1_4"] = msg->abps_batt;
    components_batteries["g3t1_5"] = msg->abpd_batt;
    components_batteries["g3t1_6"] = msg->glc_batt;
}

void DataAccess::body() {
    count_to_fetch++;
    count_to_calc_and_reset++;
    frequency = rosComponentDescriptor.getFreq();

    if (count_to_calc_and_reset >= frequency) {
        applyTimeWindow();
        for (auto component : status) {
            calculateComponentReliability(component.first);
        }

        count_to_calc_and_reset = 0;
    }

    if (count_to_fetch >= frequency*10){
        reliability_formula = fetch_formula("reliability");
        cost_formula = fetch_formula("cost");

        count_to_fetch = 0;
    }

    ros::spinOnce();
}

void DataAccess::receivePersistMessage(const archlib::Persist::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->type.c_str());
    ++logical_clock;

    if (msg->type == "Status") {
        arrived_status++;
        persistStatus(msg->timestamp, msg->source, msg->target, msg->content);
        status[msg->source].push_back({nowInSeconds(), msg->content});
    } else if (msg->type == "EnergyStatus") {
        if(msg->source != "/engine") {
            std::string component_name = msg->source;
            component_name = component_name.substr(1,component_name.size()-1);
            components_costs_engine[component_name] += std::stod(msg->content);
            components_costs_enactor[component_name] += std::stod(msg->content);
        } else {
            std::string content = msg->content;
            std::replace(content.begin(), content.end(), ';', ' ');

            std::vector<std::string> costs;
            std::stringstream ss(content);
            std::string temp;
            while(ss >> temp) costs.push_back(temp);

            std::vector<std::pair<std::string,double>> components_and_costs;
            for(std::string cost : costs) {
                std::replace(cost.begin(),cost.end(),':',' ');

                std::pair<std::string,double> temp_pair;
                std::stringstream ss(cost);
                
                ss >> temp_pair.first;
                std::string temp;
                ss >> temp;
                temp_pair.second = std::stod(temp);

                components_and_costs.push_back(temp_pair);
            }

            for(std::pair<std::string,double> component_cost : components_and_costs) {
                persistEnergyStatus(msg->timestamp, component_cost.first, msg->target, std::to_string(component_cost.second));
            }
        }
    }  else if (msg->type=="Event") {
        persistEvent(msg->timestamp, msg->source, msg->target, msg->content);
        if (events[msg->source].size()<=buffer_size) {
            events[msg->source].push_back(msg->content);
        } else {
            events[msg->source].pop_front();
            events[msg->source].push_back(msg->content);
        }
        std::string key = msg->source;
        key = key.substr(1, key.size());
        contexts[key] = msg->content == "activate" ? 1 : 0;
    } else if (msg->type=="Uncertainty") {
        persistUncertainty(msg->timestamp, msg->source, msg->target, msg->content);
    } else if (msg->type=="AdaptationCommand") {
        persistAdaptation(msg->timestamp, msg->source, msg->target, msg->content);
    } else {
        ROS_INFO("(Could not identify message type!!)");
    }
}

bool DataAccess::processQuery(archlib::DataAccessRequest::Request &req, archlib::DataAccessRequest::Response &res){
    res.content = "";

    try {
        if (req.name == "/engine" || req.name == "/enactor") {
            // wait smth like "all:status:100" -> return the last 100 success and failures of all active modules
            std::vector<std::string> query = bsn::utils::split(req.query,':');

            if (query.size() == 1){
                if (query[0] == "reliability_formula") {
                    res.content = reliability_formula;
                } else if (query[0] == "cost_formula") {
                    res.content = cost_formula;
                } 
            }

            if (query.size() > 1){
                if (query[1] == "reliability") {
                    applyTimeWindow();
                    for (auto it : status) {
                        res.content += calculateComponentReliability(it.first);
                    }
                } else if (query[1] == "event") {
                    int num = stoi(query[2]);

                    for (std::map<std::string, std::deque<std::string>>::iterator it = events.begin(); it != events.end(); it++){
                        std::string aux = it->first;
                        aux += ":";
                        bool flag = false;
                        std::string content = "";
                        for(int i = it->second.size()-num; i < it->second.size(); ++i){
                            flag = true;
                            aux += it->second[i];
                            content += it->second[i];
                            //if(i < num && i+1 < it->second.size()) 
                            aux += ",";
                        }
                        aux += ";";

                        if (flag) {
                            std::string key = it->first;
                            key = key.substr(1, key.size());
                            contexts[key] = content == "activate" ? 1 : 0;
                            res.content += aux;
                        }
                    }
                } else if (query[1] == "cost") {
                    applyTimeWindow();
                    for (auto it : status) {
                        res.content += calculateComponentCost(it.first, req.name);
                    }
                }
            }
            
        } 
    } catch(...) {}
	

    return true;
}

void DataAccess::persistEvent(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content){
    EventMessage obj("Event", timestamp, logical_clock, source, target, content);
    eventVec.push_back(obj);

    if( logical_clock % 30 == 0) flush();
}

void DataAccess::persistStatus(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content){
    StatusMessage obj("Status", timestamp, logical_clock, source, target, content);
    statusVec.push_back(obj);   

    if (logical_clock % 30 == 0) flush();
}

void DataAccess::persistEnergyStatus(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content){
    EnergyStatusMessage obj("EnergyStatus", timestamp, logical_clock, source, target, content);
    energystatusVec.push_back(obj);

    if (logical_clock % 30 == 0) flush();
}

void DataAccess::persistUncertainty(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content){
    UncertaintyMessage obj("Uncertainty", timestamp, logical_clock, source, target, content);
    uncertainVec.push_back(obj);

    if(logical_clock % 30 == 0) flush();
}

void DataAccess::persistAdaptation(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content){
    AdaptationMessage obj("Adaptation", timestamp, logical_clock, source, target, content);
    adaptVec.push_back(obj);

    if(logical_clock % 30 == 0) flush();
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

    fp.open(energy_status_filepath, std::fstream::in | std::fstream::out | std::fstream::app);   
    for(std::vector<EnergyStatusMessage>::iterator it = energystatusVec.begin(); it != energystatusVec.end(); ++it) {
        fp << (*it).getName() << ",";
        fp << (*it).getLogicalClock() << ",";
        fp << (*it).getTimestamp() << ",";
        fp << (*it).getSource() << ",";
        fp << (*it).getTarget() << ",";
        fp << (*it).getCost() << "\n";
    }
    fp.close();
    energystatusVec.clear();

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

/**
 * Builds the response string and calculate the reliability
 * of the component specified as parameter
*/
std::string DataAccess::calculateComponentReliability(const std::string& component) {
    std::string aux = component, content = "";
    aux += ":";
    bool flag = false;
    double sum = 0;
    uint32_t len = 0;
    for (auto value : status[component]) {
        // reliability = success/(success + fails)
        if (value.second == "success") { // calculate reliability
            sum +=1;
            len++;
        } else if (value.second == "fail") {
            len++;
        }
        else { // it can be other values
            aux += value.second;
            aux += ",";
        }
        
        flag = true;
    }
    aux += std::to_string((len > 0) ? sum / len : 0) + ';';

    std::string key = component;
    key = key.substr(1, key.size());

    if(flag) content += aux;

    components_reliabilities[key] = (len > 0) ? sum / len : 0;

    return content;
}

/**
 * Builds the response string and calculate the cost
 * of the component specified as parameter
*/
std::string DataAccess::calculateComponentCost(const std::string& component, std::string req_name) {
    std::string aux = component;
    aux += ":";

    std::string key = component;
    key = key.substr(1, key.size());

    if(req_name == "/engine") {
        aux += std::to_string(components_costs_engine[key]) + ';';
        components_costs_engine[key] = 0;
    } else if(req_name == "/enactor") {
        aux += std::to_string(components_costs_enactor[key]) + ';';
        components_costs_enactor[key] = 0;
    }

    return aux;
}

void DataAccess::applyTimeWindow() {
    auto now = nowInSeconds();

    for (auto& component : status) {
        auto& deq = component.second;
        while (!deq.empty()) {
            auto time_arrived = deq.front().first;
            double time_span = std::chrono::duration_cast<std::chrono::duration<double>>(now - time_arrived).count();
            if (time_span >= 10.1) {
                deq.pop_front();
            } else {
                component.second = deq;
                break;
            }
        }
    }
}