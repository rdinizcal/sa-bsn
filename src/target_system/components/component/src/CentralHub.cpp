#include "component/CentralHub.hpp"

CentralHub::CentralHub(int &argc, char **argv, const std::string &name, const bool &active, const bsn::resource::Battery &battery) : Component(argc, argv, name), active(active), max_size(5), total_buffer_size(0), buffer_size({0,0,0,0,0}), battery(battery), data_buffer({{0},{0},{0},{0},{0}}) {}

CentralHub::~CentralHub() {}

int32_t CentralHub::run() {
	setUp();

    ros::NodeHandle nh;
    ros::Subscriber thermometerSub = nh.subscribe("thermometer_data", 10, &CentralHub::collect, this);
    ros::Subscriber oximeterSub = nh.subscribe("oximeter_data", 10, &CentralHub::collect, this);
    ros::Subscriber ecgSub = nh.subscribe("ecg_data", 10, &CentralHub::collect, this);
    ros::Subscriber diastolicSub = nh.subscribe("diastolic_data", 10, &CentralHub::collect, this);
    ros::Subscriber systolicSub = nh.subscribe("systolic_data", 10, &CentralHub::collect, this);

    while(ros::ok()) {
        ros::Rate loop_rate(rosComponentDescriptor.getFreq());

        try{
            body();
        } catch (const std::exception& e) {
            sendStatus("fail");
        } 
        loop_rate.sleep();
    }

    return 0;
}

void CentralHub::body() {
    
    ros::spinOnce(); //calls collect() if there's data in the topics

    if (!isActive() && battery.getCurrentLevel() > 90){
        turnOn();
    } else if (isActive() && battery.getCurrentLevel() < 2){
        turnOff();        
    }
    
    if(isActive()) {
        if(total_buffer_size > 0){
            apply_noise();
            process();
            transfer();
            sendStatus("success");
        }
    } else {
        recharge();
        throw std::domain_error("out of charge");
    }
}

void CentralHub::apply_noise() {}


void CentralHub::reconfigure(const archlib::AdaptationCommand::ConstPtr& msg) {
    std::string action = msg->action.c_str();

    bsn::operation::Operation op;
    std::vector<std::string> pairs = op.split(action, ',');

    for (std::vector<std::string>::iterator it = pairs.begin(); it != pairs.end(); ++it){
        std::vector<std::string> param = op.split(action, '=');

        if(param[0]=="replicate_collect"){
            rosComponentDescriptor.setFreq(rosComponentDescriptor.getFreq()+stoi(param[1]));
        }

    }
}

bool CentralHub::isActive() {
    return active;
}

void CentralHub::turnOn() {
    active = true;
    activate();
}

void CentralHub::turnOff() {
    active = false;
    deactivate();
}

void CentralHub::recharge() {
    if(battery.getCurrentLevel() <= 100) 
        battery.generate((100/20)/rosComponentDescriptor.getFreq());
}