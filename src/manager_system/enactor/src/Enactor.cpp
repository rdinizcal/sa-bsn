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
        replicate_task.erase(msg->source);
    }

}

void Enactor::receiveStatus(const archlib::Status::ConstPtr& msg) {

    if (msg->content=="init") {

        executions[msg->source] = {};
        reliability[msg->source] = 1;
        replicate_task[msg->source] = 1;

    } else if (msg->content=="success") {
        
        if(executions[msg->source].size() < 100) {
            executions[msg->source].push_back(1);
        } else {
            executions[msg->source].pop_front();
            executions[msg->source].push_back(1);

            int sum = 0;
            for(std::deque<int>::iterator it = executions[msg->source].begin(); it != executions[msg->source].end(); ++it) {
                sum += (*it);
            }

            reliability[msg->source] = double(sum)/double(executions[msg->source].size());

            apply_strategy(msg->source);
        }

    } else if (msg->content=="fail") {
        //apply strategy only if you have at least 10 executions information
        if(executions[msg->source].size() < 100) {
            executions[msg->source].push_back(0);
        } else {
            executions[msg->source].pop_front();
            executions[msg->source].push_back(0);

            int sum = 0;
            for(std::deque<int>::iterator it = executions[msg->source].begin(); it != executions[msg->source].end(); ++it) {
                sum += (*it);
            }

            reliability[msg->source] = double(sum)/double(executions[msg->source].size());

            apply_strategy(msg->source);
        }
    }
}

void Enactor::apply_strategy(const std::string &component) {

    /*if(reliability[component]<0.8) {
        replicate_task[(component)] += 10;
    } else if (reliability[component] > 0.95 && replicate_task[(component)] > 1) {
        replicate_task[(component)] -= 1;
    } else {
        return;
    }*/
    std::cout << "reli[" << component << "] = "<< reliability[component] <<std::endl;
    double Kp = 200;
    double reference = 0.80;
    double error = reference - reliability[component]; //error = Rref - Rcurr
    replicate_task[component] += (error>0)?ceil(Kp * error):floor(Kp*error);

    if(replicate_task[component] < 1) replicate_task[component]=1;

    archlib::AdaptationCommand msg;

    msg.source = ros::this_node::getName();
    msg.target = component;
    msg.action = "replicate_collect=" + std::to_string(replicate_task[(component)]);

    adapt.publish(msg);

    executions[component].clear();
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
        std::cout << "  - buffer size: " << replicate_task[*it] << std::endl;
        std::cout << "****************************************" << std::endl;
    }
}