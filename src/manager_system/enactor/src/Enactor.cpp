#include "enactor/Enactor.hpp"

Enactor::Enactor(int  &argc, char **argv, std::string name) : ROSComponent(argc, argv, name), cycles(0) {}

Enactor::~Enactor() {}

void Enactor::setUp() {
    ros::NodeHandle nh;

    adapt = nh.advertise<archlib::AdaptationCommand>("log_adapt", 10);

    double freq;
	nh.getParam("frequency", freq);
	rosComponentDescriptor.setFreq(freq);
}

void Enactor::tearDown() {}

void Enactor::receiveEvent(const archlib::Event::ConstPtr& msg) {

    if (msg->content=="activated") {

        connected.push_back(msg->source);

    } else if (msg->content=="deactivated") {

        for(std::vector<std::string>::iterator it = connected.begin(); it != connected.end(); ++it) {
            if((*it)==msg->source) {
                connected.erase(it);
                break;
            }
        }

        executions.erase(msg->source);
        reliability.erase(msg->source);
        buffer_size.erase(msg->source);
    }

}

void Enactor::receiveStatus(const archlib::Status::ConstPtr& msg) {

    if (msg->content=="init") {

        executions[msg->source] = {};
        reliability[msg->source] = 1;
        buffer_size[msg->source] = 1;

    } else if (msg->content=="success") {
        
        if(executions[msg->source].size() < 5) {
            executions[msg->source].push_back(1);
            std::cout << msg->source <<"  - executions: [";
            for(std::deque<int>::iterator itt = executions[msg->source ].begin(); itt != executions[msg->source ].end(); ++itt) std::cout << *itt << " ";
            std::cout << "]" << std::endl;
        } else {
            executions[msg->source].pop_front();
            executions[msg->source].push_back(1);

            int sum = 0;
            for(std::deque<int>::iterator it = executions[msg->source].begin(); it != executions[msg->source].end(); ++it) {
                sum += (*it);
            }

            reliability[msg->source] = sum/executions[msg->source].size();

            apply_strategy(msg->source);
        }

    } else if (msg->content=="fail") {
        //apply strategy only if you have at least 10 executions information
        if(executions[msg->source].size() < 5) {
            executions[msg->source].push_back(0);
            std::cout << msg->source << "  - executions: [";
            for(std::deque<int>::iterator itt = executions[msg->source ].begin(); itt != executions[msg->source ].end(); ++itt) std::cout << *itt << " ";
            std::cout << "]" << std::endl;
        } else {
            executions[msg->source].pop_front();
            executions[msg->source].push_back(0);

            int sum = 0;
            for(std::deque<int>::iterator it = executions[msg->source].begin(); it != executions[msg->source].end(); ++it) {
                sum += (*it);
            }

            reliability[msg->source] = sum/executions[msg->source].size();

            apply_strategy(msg->source);
        }
    }
}

void Enactor::apply_strategy(const std::string &component) {

    print();

    bool strategy_condition = false;
    if (reliability[component]<0.8) strategy_condition = true; 

    if(strategy_condition) {

        if(buffer_size[(component)] <= 88) buffer_size[(component)] += 2;
        else buffer_size[(component)] = 100;

        archlib::AdaptationCommand msg;

        msg.source = ros::this_node::getName();
        msg.target = component;
        msg.action = "buffer_size=" + std::to_string(buffer_size[(component)]);

        adapt.publish(msg);

        executions[component].clear();
    }

    std::vector<std::string> reliable_components;
    for (std::map<std::string, double>::iterator it = reliability.begin(); it != reliability.end(); ++it){
        if((*it).second==1 && buffer_size[(*it).first] > 1) reliable_components.push_back((*it).first);
    }
    
    for (std::vector<std::string>::iterator it = reliable_components.begin(); it != reliable_components.end(); ++it){
        buffer_size[(*it)] -= 1;
        archlib::AdaptationCommand msg;

        msg.source = ros::this_node::getName();
        msg.target = *it;
        msg.action = "buffer_size=" + std::to_string(buffer_size[(*it)]);

        adapt.publish(msg);
    }

}

void Enactor::body(){
    ros::NodeHandle n;
    ros::Subscriber subs_event = n.subscribe("event", 1000, &Enactor::receiveEvent, this);
    ros::Subscriber subs_status = n.subscribe("status", 1000, &Enactor::receiveStatus, this);
    ros::spin();

    ++cycles;
}

void Enactor::print() {

    for(std::vector<std::string>::iterator it = connected.begin(); it != connected.end(); ++it) {
        std::cout << "****************************************" << std::endl;
        std::cout << *it << std::endl;
        std::cout << "  - executions: [";
        for(std::deque<int>::iterator itt = executions[*it].begin(); itt != executions[*it].end(); ++itt) std::cout << *itt << " ";
        std::cout << "]" << std::endl;
        std::cout << "  - reliability: " << reliability[*it] << std::endl;
        std::cout << "  - buffer size: " << buffer_size[*it] << std::endl;
        std::cout << "****************************************" << std::endl;
    }
}