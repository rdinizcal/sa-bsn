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

    info_dataaccessnode2learning_pub = handle.advertise<messages::LearningData>("learning_info", 10);
    info_dataaccessnode2controller_pub = handle.advertise<messages::ControllerInfo>("controller_info", 10);

    ROS_INFO("Running DataAccessNode\n");
}

void DataAccessNode::tearDown() {}

void DataAccessNode::run(){
    ros::Rate loop_rate(10);
    ros::NodeHandle n;
    ros::Subscriber receive_data = n.subscribe("persist", 1000, &DataAccessNode::receiveInfo, this);

    ros::spin();
}

void DataAccessNode::sendControllerInfo() {
    messages::ControllerInfo info;

    std::map<std::string,std::vector<ComponentData>>::iterator it;

    int i = 0;

    for(it = componentMap.begin();it != componentMap.end();++it) {
        if(it->second.back().getType() != "centralhub") { //Test purposes, centralhub does not send relevant controller info until now!
            info.module_name.push_back(it->first);

            ComponentData component = it->second.back();

            info.battery_level.push_back(component.getBatteryLevel());
            info.cost.push_back(component.getCost());
            info.frequency.push_back(component.getFrequency());
            info.risk_status.push_back(component.getRiskStatus());

            i++;
        }
    }

    if(i == 0) {
        info.empty = true;
    } else {
        info.empty = false;
    }

    info_dataaccessnode2controller_pub.publish(info);
}

void DataAccessNode::parse(std::string content, ComponentData& component) {
    std::string aux = "";
    bool isValue = false;
    int valueCounter = 0;

    char separator = ',';
    char delimiter = ':';

    double valueHolder;

    for(std::string::iterator it = content.begin();it != content.end();++it) {
        if((*it) == separator || std::next(it) == content.end()) {
            //Component data filling
            if(std::next(it) == content.end()) {
                aux += (*it);
            }

            if(aux.size() > 0) {
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

void DataAccessNode::finishCycle(double &timestamp) {
    std::map<std::string,std::vector<ComponentData>>::iterator componentMapIterator;
    std::vector<std::string>::iterator componentsInCycleIterator;

    for(componentMapIterator = componentMap.begin();componentMapIterator != componentMap.end();++componentMapIterator) {
        componentsInCycleIterator = std::find(components_in_cycle.begin(), components_in_cycle.end(), componentMapIterator->first);

        if(componentsInCycleIterator == components_in_cycle.end() && componentMapIterator->second.size() > 0) {
            ComponentData component(componentMapIterator->second.back());

            component.setTimestamp(timestamp);
        
            componentMapIterator->second.push_back(component);
        }
    }

    components_in_cycle.clear();
}

void DataAccessNode::receiveInfo(const messages::Info::ConstPtr& msg) {
    ComponentData componentData;
    std::vector<std::string>::iterator componentsInCycleIterator;

    parse(msg->content, componentData);

    ROS_INFO("I heard: [%s]", msg->content.c_str());

    int size = componentMap[componentData.getName()].size();
    
    if(size == 0) { //Making vector sizes equal
        int max_size = 0;
        std::map<std::string,std::vector<ComponentData>>::iterator componentMapIterator;
        for(componentMapIterator = componentMap.begin();componentMapIterator != componentMap.end();++componentMapIterator) {
            if(componentMapIterator->second.size() > max_size) {
                max_size = componentMapIterator->second.size();
            }
        }

        if(max_size > 1) {
            ComponentData dummy_data(componentData);

            for(int i = 0;i < max_size-1;i++) {
                componentMap[componentData.getName()].push_back(dummy_data);
            }
        }
    }

    double timestamp;

    componentsInCycleIterator = std::find(components_in_cycle.begin(), components_in_cycle.end(), componentData.getName());

    if(componentsInCycleIterator != components_in_cycle.end()) {
        timestamp = componentMap[componentData.getName()][size-1].getTimestamp();
        finishCycle(timestamp);
    }

    componentMap[componentData.getName()].push_back(componentData);

    components_in_cycle.push_back(componentData.getName());

    logical_clock++;

    if(logical_clock % 2000 == 0) { //Maybe change size
        timestamp = componentData.getTimestamp();
        finishCycle(timestamp);
        persistComponentData();
    }
}

void DataAccessNode::sendLearningInfo() {
    //Send info to learning block in a specific way
    std::map<std::string,std::vector<ComponentData>>::iterator componentMapIterator;
    messages::LearningData learning_data;
    
    int i = 0;

    componentMapIterator = componentMap.begin();

    int size = componentMapIterator->second.size();

    std::string aux;

    while(i < size) {
        for(componentMapIterator = componentMap.begin();componentMapIterator != componentMap.end();++componentMapIterator) {
            if(componentMapIterator->second.back().getType() != "centralhub") {
                if(i == 0) {
                    learning_data.types.push_back(componentMapIterator->second.at(i).getType());
                }

                learning_data.type.push_back(componentMapIterator->second.at(i).getType());

                learning_data.risk_status.push_back(componentMapIterator->second.at(i).getRiskStatus());
            } else {
                aux = componentMapIterator->first;
            }
        }
        
        componentMapIterator = componentMap.find(aux);

        if(componentMapIterator != componentMap.end()) {
            if(i == 0) {
                learning_data.types.push_back(componentMapIterator->second.at(i).getType());
            }

            learning_data.type.push_back(componentMapIterator->second.at(i).getType());
            learning_data.risk_status.push_back(componentMapIterator->second.at(i).getRiskStatus());
        }
        
        i++;
    }

    info_dataaccessnode2learning_pub.publish(learning_data);
}

void DataAccessNode::persistComponentData() {
    std::map<std::string,std::vector<ComponentData>>::iterator componentMapIterator;
    std::vector<ComponentData>::iterator componentDataVectorIterator;

    sendControllerInfo();
    sendLearningInfo();

    fp.open(filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);

    for(componentMapIterator = componentMap.begin();componentMapIterator != componentMap.end();++componentMapIterator) {
        for(componentDataVectorIterator = componentMapIterator->second.begin();componentDataVectorIterator != componentMapIterator->second.end();++componentDataVectorIterator) {
            fp << (*componentDataVectorIterator).toString();
        }
        componentMap.erase(componentMapIterator);
    }

    fp.close();
}

int64_t DataAccessNode::now() const{
    return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}