#include "data_access_node/DataAccessNode.hpp"

DataAccessNode::DataAccessNode(int  &argc, char **argv) :
    filepath(""),
    logical_clock(0),
    first_persist(false) {}

DataAccessNode::~DataAccessNode() {}

void DataAccessNode::setUp() {
    std::string path = ros::package::getPath("illness_identifier");
    //filepath = path + "/DataAccessNode-" + std::to_string(this->now()) + ".csv";

    /*const char *homedir = getenv("HOME");
		if(homedir == NULL) {
			homedir = getpwuid(getuid())->pw_dir;
		}

    std::string path(homedir);*/

    filepath += path;

    filepath += "/src/DataAccessNodeData.csv";

    fp.open(filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    //fp << "timestamp module_name type battery_level frequency cost risk_status\n";
    //fp.close();

    info_dataaccessnode2learning_pub = handle.advertise<messages::LearningData>("learning_info", 10);
    info_dataaccessnode2controller_pub = handle.advertise<messages::ControllerInfo>("controller_info", 10);

    ROS_INFO("Running DataAccessNode\n");
}

void DataAccessNode::tearDown() {
    fp.close();
}

void DataAccessNode::run(){
    ros::Rate loop_rate(10);
    ros::NodeHandle n;
    ros::Subscriber receive_data = n.subscribe("persist", 1000, &DataAccessNode::receiveInfo, this);

    ros::spin();

    return tearDown();
}

void DataAccessNode::sendControllerInfo() {
    messages::ControllerInfo info;

    std::map<std::string,ComponentData>::iterator it;

    int i = 0;

    for(it = componentMap.begin();it != componentMap.end();++it) {
        if(it->second.getType() != "centralhub") { //Test purposes, centralhub does not send relevant controller info until now!
            info.module_name.push_back(it->first);

            ComponentData component = it->second;

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
    std::string component_type = "";
    bool isValue = false;
    bool isType = true;
    int valueCounter = 0;

    char separator = ',';
    char delimiter = ':';
    char type_separator = '_';

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
                        if(component.getType() != "centralhub") {
                            component.setRiskStatus(aux);
                        }
                }

                /*if(component.getType() != "centralhub" && valueCounter == 6) {
                    std::cout << "##################################### INFO ######################################" << std::endl;
                    std::cout << "content: " << content << std::endl;
                    std::cout << "timestamp: " << component.getTimestamp() << std::endl;
                    std::cout << "name: " << component.getName() << std::endl;
                    std::cout << "type: " << component.getType() << std::endl;
                    std::cout << "battery: " << component.getBatteryLevel() << std::endl;
                    std::cout << "frequency: " << component.getFrequency() << std::endl;
                    std::cout << "cost: " << component.getCost() << std::endl;
                    std::cout << "risk: " << component.getRiskStatus() << std::endl;
                    std::cout << "#################################################################################" << std::endl;
                }*/

                if(component.getType() == "centralhub" && valueCounter >= 6) {
                    if(component_type == "centralhub") {
                        component.setRiskStatus(aux);
                    }

                    std::map<std::string, std::vector<std::string>>::iterator risks_iterator;
                    bool no_entry = false;

                    risks_iterator = risks.find(component_type);
                    
                    size_t this_size = 0;

                    if(risks_iterator == risks.end()) {
                        no_entry = true;     
                    } else {
                        this_size = risks_iterator->second.size();
                    }

                    size_t max_size = -1;

                    for(risks_iterator = risks.begin();risks_iterator != risks.end();++risks_iterator) {
                        if(risks_iterator->second.size() > max_size) {
                            max_size = risks_iterator->second.size();
                        }
                    }

                    if(this_size < max_size - 1) {
                        int diff = max_size - this_size;
                        std::string stat = "undefined";
                        for(int j = 0;j < diff;j++) {
                            risks[component_type].push_back(stat);
                        }
                    }

                    risks[component_type].push_back(aux);
                }
            }

            valueCounter++;
            aux.clear();
            component_type.clear();
            isValue = false;
            isType = true;
        }

        if(isValue) {
            aux += (*it);
        } else {
            if((*it) != separator) {
                if(isType) {
                    if((*it) != type_separator) {
                        component_type += (*it);
                    } else {
                        isType = false;
                    }
                }
            }
        }

        if((*it) == delimiter) {
            isValue = true;
        }
    }
}

void DataAccessNode::receiveInfo(const messages::Info::ConstPtr& msg) {
    ComponentData componentData;

    parse(msg->content, componentData);

    ROS_INFO("I heard: [%s]", msg->content.c_str());

    if(componentData.getRiskStatus() != "undefined") {
        componentMap[componentData.getName()] = componentData;
    } else {
        std::string actual_risk = componentMap[componentData.getName()].getRiskStatus();
        componentData.setRiskStatus(actual_risk);
        componentMap[componentData.getName()] = componentData;
    }

    logical_clock++;

    if(logical_clock % 100 == 0) { //Maybe change size
        persistComponentData();
    }
}

void DataAccessNode::persistComponentData() {
    sendControllerInfo();

    std::string aux;

    std::map<std::string, std::vector<std::string>>::iterator risks_iterator;
    
    if(first_persist == false) {
        for(risks_iterator = risks.begin();risks_iterator != risks.end();++risks_iterator) {
            if(risks_iterator->first != "centralhub") {
                fp << risks_iterator->first << "_DATA,";
            } else {
                aux = risks_iterator->first;
            }
        }

        components_to_persist.push_back(aux);
        fp << "PATIENT_STATE\n";

        first_persist = true;
    }

    risks_iterator = risks.begin();

    int size = risks_iterator->second.size();

    std::string r;

    int index = 0;

    while(index < size) {
        for(risks_iterator = risks.begin();risks_iterator != risks.end();++risks_iterator) {
            if(risks_iterator->first != "centralhub") {
                fp << risks_iterator->second.at(index) << ",";
            } else {
                r = risks_iterator->second.at(index);
            }
        }
        fp << r << "\n";
        index++;

    }

    for(risks_iterator = risks.begin();risks_iterator != risks.end();++risks_iterator) {
        risks.erase(risks_iterator);
    }
}

int64_t DataAccessNode::now() const{
    return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}