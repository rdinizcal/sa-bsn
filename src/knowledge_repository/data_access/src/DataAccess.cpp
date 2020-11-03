#include "data_access/DataAccess.hpp"

#define W(x) std::cerr << #x << " = " << x << std::endl;

DataAccess::DataAccess(int  &argc, char **argv, const std::string &name) : ROSComponent(argc, argv, name), fp(), event_filepath(), status_filepath(), logical_clock(0), statusVec(), eventVec(), status(), buffer_size() {}
DataAccess::~DataAccess() {}

int64_t DataAccess::now() const{
    return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

std::chrono::high_resolution_clock::time_point DataAccess::nowInSeconds() const {
    return std::chrono::high_resolution_clock::now();
}

void DataAccess::setUp() {
    std::string path = ros::package::getPath("repository");
    std::string url;
    file_id = std::to_string(this->now());

    event_filepath = path + "/../resource/logs/event_" + file_id + ".log";
    tmp_event_filepath = path + "/../resource/logs/event_" + file_id + "_tmp.log";
    status_filepath = path + "/../resource/logs/status_" + file_id + ".log";
    tmp_status_filepath = path + "/../resource/logs/status_" + file_id + "_tmp.log";
    uncertainty_filepath = path + "/../resource/logs/uncertainty_" + file_id + ".log";
    adaptation_filepath = path + "/../resource/logs/adaptation_" + file_id + ".log";
    ctmetrics_filepath = path + "/../resource/logs/ctmetrics_" + file_id + ".log";
    engineinfo_filepath = path + "/../resource/logs/engineinfo_" + file_id + ".log";
    tsdata_filepath = path + "/../resource/logs/targetsystemdata_" + file_id + ".json";

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

    fp.open(ctmetrics_filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "\n";
    fp.close();

    fp.open(engineinfo_filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "\n";
    fp.close();

    fp.open(tsdata_filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "\n";
    fp.close();


	handle.getParam("frequency", frequency);
    rosComponentDescriptor.setFreq(frequency);

    handle.getParam("server_url", url);

    buffer_size = 1000;

    handle.getParam("connect", connected);

    std::ifstream reliability_file, cost_file;
    std::string reliability_formula, cost_formula;

    try {
        reliability_file.open(path + "/../resource/models/reliability.formula");
        std::getline(reliability_file, reliability_formula);
        reliability_file.close();
    } catch (std::ifstream::failure e) {
        std::cerr << "Exception opening/reading/closing file (reliability.formula)\n";
    }

    try {
        cost_file.open(path + "/../resource/models/cost.formula");
        std::getline(cost_file, cost_formula);
        cost_file.close();
    } catch (std::ifstream::failure e) {
        std::cerr << "Exception opening/reading/closing file (cost.formula)\n";
    }

    cost_expression = bsn::model::Formula(cost_formula);
    reliability_expression = bsn::model::Formula(reliability_formula);

    this->client = std::shared_ptr<web::http::client::http_client>
        (new web::http::client::http_client(U(url)));

    count_to_calc_and_reset = 0;
    arrived_status = 0;

    components_batteries["g3t1_1"] = 100;
    components_batteries["g3t1_2"] = 100;
    components_batteries["g3t1_3"] = 100;
    components_batteries["g3t1_4"] = 100;
    components_batteries["g3t1_5"] = 100;
    components_costs["g3t1_1"] = 0;
    components_costs["g3t1_2"] = 0;
    components_costs["g3t1_3"] = 0;
    components_costs["g3t1_4"] = 0;
    components_costs["g3t1_5"] = 0;
    components_reliabilities["g3t1_1"] = 1;
    components_reliabilities["g3t1_2"] = 1;
    components_reliabilities["g3t1_3"] = 1;
    components_reliabilities["g3t1_4"] = 1;
    components_reliabilities["g3t1_5"] = 1;

    
    handle_persist = handle.subscribe("persist", 1000, &DataAccess::receivePersistMessage, this);
    server = handle.advertiseService("DataAccessRequest", &DataAccess::processQuery, this);
    adr_server = handle.advertiseService("address", &DataAccess::sendAddress, this);
   
    targetSystemSub = handle.subscribe("TargetSystemData", 100, &DataAccess::processTargetSystemData, this);
}

void DataAccess::tearDown(){}

void DataAccess::processTargetSystemData(const messages::TargetSystemData::ConstPtr &msg) {
    if (connected) {
        components_batteries["g3t1_3"] = msg->trm_batt;
        components_batteries["g3t1_2"] = msg->ecg_batt;
        components_batteries["g3t1_1"] = msg->oxi_batt;
        components_batteries["g3t1_4"] = msg->abps_batt;
        components_batteries["g3t1_5"] = msg->abpd_batt;

        web::json::value json_obj, sensorPacket, patientPacket, reli_costPacket;

        sensorPacket["battery"] = msg->trm_batt;
        sensorPacket["risk"] = msg->trm_risk;
        sensorPacket["raw"] = msg->trm_data;
        json_obj["ThermometerPacket"] = sensorPacket;

        sensorPacket["battery"] = msg->ecg_batt;
        sensorPacket["risk"] = msg->ecg_risk;
        sensorPacket["raw"] = msg->ecg_data;
        json_obj["EcgPacket"] = sensorPacket;

        sensorPacket["battery"] = msg->oxi_batt;
        sensorPacket["risk"] = msg->oxi_risk;
        sensorPacket["raw"] = msg->oxi_data;
        json_obj["OximeterPacket"] = sensorPacket;

        sensorPacket["battery"] = msg->abps_batt;
        sensorPacket["risk"] = msg->abps_risk;
        sensorPacket["raw"] = msg->abps_data;
        json_obj["ABPSPacket"] = sensorPacket;

        sensorPacket["battery"] = msg->abpd_batt;
        sensorPacket["risk"] = msg->abpd_risk;
        sensorPacket["raw"] = msg->abpd_data;
        json_obj["ABPDPacket"] = sensorPacket;

        json_obj["PatientRisk"] = msg->patient_status;

        json_obj["Reliability"] = system_reliability * 100;
        json_obj["Cost"] = system_cost;
        client->request(web::http::methods::POST, U("/sendVitalData"), json_obj);
        ROS_INFO("Sent information to server.");
    /*
        fp.open(tsdata_filepath, std::fstream::in | std::fstream::out | std::fstream::app);
        fp << json_obj << std::endl;
        fp.close();
    */
    }
}

double DataAccess::calculateCost() {
    std::vector<std::string> keys;
    std::vector<double> values;
    std::string context_key, formated_key, r_key, w_key, f_key;

    for (auto x : components_costs) {
        formated_key = x.first;
        std::transform(formated_key.begin(), formated_key.end(), formated_key.begin(), ::toupper);
        formated_key.insert(int(formated_key.find('T')), "_");

        w_key = "W_" + formated_key;
        keys.push_back(w_key);
        values.push_back(x.second);
    }

    for (auto x : contexts) {
        formated_key = x.first;
        std::transform(formated_key.begin(), formated_key.end(), formated_key.begin(), ::toupper);
        formated_key.insert(int(formated_key.find('T')), "_");

        context_key = "CTX_" + formated_key;
        keys.push_back(context_key);
        values.push_back(x.second);

        r_key = "R_" + formated_key;
        keys.push_back(r_key);
        values.push_back(1);

        f_key = "F_" + formated_key;
        keys.push_back(f_key);
        values.push_back(1);
    }

    return cost_expression.apply(keys, values);
}

bool DataAccess::sendAddress(services::Address::Request &req, services::Address::Response &res){
    ROS_INFO("Received address request--------------------------------");
    res.id = file_id;
    return true;
}

double DataAccess::calculateReliability() {
    std::vector<std::string> keys;
    std::vector<double> values;
    std::string context_key, formated_key, r_key, f_key;

    for (auto x : components_reliabilities) {
        formated_key = x.first;
        std::transform(formated_key.begin(), formated_key.end(), formated_key.begin(), ::toupper);
        formated_key.insert(int(formated_key.find('T')), "_");

        r_key = "R_" + formated_key;
        keys.push_back(r_key);
        values.push_back(x.second);
    }

    for (auto x : contexts) {
        formated_key = x.first;
        std::transform(formated_key.begin(), formated_key.end(), formated_key.begin(), ::toupper);
        formated_key.insert(int(formated_key.find('T')), "_");

        context_key = "CTX_" + formated_key;
        keys.push_back(context_key);
        values.push_back(x.second);

        f_key = "F_" + formated_key;
        keys.push_back(f_key);
        values.push_back(1);
    }

    return reliability_expression.apply(keys, values);
}

void DataAccess::body() {
    count_to_calc_and_reset++;
    frequency = rosComponentDescriptor.getFreq();

    if (count_to_calc_and_reset >= frequency) {
        applyTimeWindow();
        for (auto component : status) {
            calculateComponentReliability(component.first);
        }
        system_reliability = calculateReliability();
        updateCosts();
        system_cost = calculateCost();
        updateBatteries();
        count_to_calc_and_reset = 0;
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
    } else if (msg->type=="Event") {
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
    } else if(msg->type=="EngineInfo") {
        persistEngineInfo(msg->timestamp, msg->source, msg->target, msg->content);
    } else if (msg->type=="ControlTheoryMetrics") {
        persistControlTheoryMetrics(msg->timestamp, msg->source, msg->target, msg->content);
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
            }
        } 
    } catch(...) {}
	

    return true;
}

void DataAccess::persistEvent(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content){
    EventMessage obj("Event", timestamp, logical_clock, source, target, content);
    eventVec.push_back(obj);
    eventVecTmp.push_back(obj);

    if( logical_clock % 30 == 0) flush();
}

void DataAccess::persistStatus(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content){
    StatusMessage obj("Status", timestamp, logical_clock, source, target, content);
    statusVec.push_back(obj);
    statusVecTmp.push_back(obj);

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

void DataAccess::persistControlTheoryMetrics(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content){
    
    std::vector<std::string> metrics = bsn::utils::split(content, ';');

    ControlTheoryMetricsMessage obj("ControlTheoryMetrics", timestamp, logical_clock, source, target, metrics[0], metrics[1], metrics[2], metrics[3], metrics[4], metrics[5]);
    ctmetricsVec.push_back(obj);

    if(logical_clock % 30 == 0) flush();
}

void DataAccess::persistEngineInfo(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content) {
 
    std::vector<std::string> data = bsn::utils::split(content, ';');

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

void DataAccess::updateBatteries() {
    for (auto component : components_batteries) {
        components_last_batteries[component.first] = component.second;
    }
}

void DataAccess::updateCosts() {
    for (auto component : components_batteries) {
        components_costs[component.first] = 
            components_last_batteries[component.first] - component.second;
    }
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