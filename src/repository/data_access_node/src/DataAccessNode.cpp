#include "data_access_node/DataAccessNode.hpp"

DataAccessNode::DataAccessNode(int  &argc, char **argv) :
    filepath(""),
    logical_clock(0) {}

DataAccessNode::~DataAccessNode() {}

void DataAccessNode::setUp() {
    std::string path = ros::package::getPath("logger");
    filepath = path + "/resource/logs/DataAccessNode-" + std::to_string(this->now()) + ".log";

    fp.open(filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "timestamp module_name type battery_level frequency cost risk_status\n";
    fp.close();

    info_service = handle.advertiseService("InfoRequest", &DataAccessNode::sendInfo, this);
}

void DataAccessNode::tearDown() {}

void DataAccessNode::run(){
    ros::Rate loop_rate(100);
    ros::NodeHandle n;
    ros::Subscriber receive_data = n.subscribe("persist", 1000, &DataAccessNode::receiveInfo, this);
    
    while(ros::ok()) {

        
        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool DataAccessNode::sendInfo(services::ControllerInfo::Request &req, services::ControllerInfo::Response &res) {
    std::map<std::string,std::vector<ComponentData>>::iterator it;

    int i = 0;

    for(it = componentMap.begin();it != componentMap.end();++it) {
        res.module_name[i] = it->first;

        ComponentData component = it->second.back();

        res.battery_level[i] = component.getBatteryLevel();
        res.cost[i] = component.getCost();
        res.frequency[i] = component.getFrequency();
        res.risk_status[i] = component.getRiskStatus();

        i++;
    }

    if(i == 0) {
        res.empty = true;
    } else {
        res.empty = false;
    }

    return true;
}

void DataAccessNode::parse(std::string content, ComponentData& component) {
    std::string aux = "";
    bool isValue = false;
    int valueCounter = 0;

    char separator = ',';
    char delimiter = ':';

    double valueHolder;

    for(std::string::iterator it = content.begin();it != content.end();++it) {
        if((*it) == separator) {
            //Component data filling
            switch(valueCounter) {
                case 0:
                    valueHolder = std::stod(aux);
                    component.setTimestamp(valueHolder);
                    break;
                case 1:
                    component.setName(aux);
                    break;
                case 2:
                    component.setType(aux);
                    break;
                case 3:
                    valueHolder = std::stod(aux);
                    component.setBatteryLevel(valueHolder);
                    break;
                case 4:
                    valueHolder = std::stod(aux);
                    component.setFrequency(valueHolder);
                    break;
                case 5:
                    valueHolder = std::stod(aux);
                    component.setCost(valueHolder);
                    break;
                case 6:
                    component.setRiskStatus(aux);
            }

            valueCounter++;
            aux.clear();
            isValue = false;
        }

        if(isValue) {
            aux += (*it);
        }

        if((*it) == delimiter) {
            isValue = true;
        }
    }
} 

void DataAccessNode::receiveInfo(const messages::Info::ConstPtr& msg) {
    ComponentData componentData;

    parse(msg->content, componentData);

    componentMap[componentData.getName()].push_back(componentData);

    logical_clock++;
    
    if(logical_clock % 100 == 0) {
        persistComponentData();
    }
}

void DataAccessNode::persistComponentData() {
    std::map<std::string,std::vector<ComponentData>>::iterator componentMapIterator;
    std::vector<ComponentData>::iterator componentDataVectorIterator;

    fp.open(filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);

    for(componentMapIterator = componentMap.begin();componentMapIterator != componentMap.end();++componentMapIterator) {
        for(componentDataVectorIterator = componentMapIterator->second.begin();componentDataVectorIterator != componentMapIterator->second.end();++componentDataVectorIterator) {
            fp << (*componentDataVectorIterator).toString();
        }
        componentMapIterator->second.clear();
    }

    fp.close();
}

int64_t DataAccessNode::now() const{
    return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}