#include "component/CentralHub.hpp"

CentralHub::CentralHub(int &argc, char **argv, const std::string &name, const bool &active, const bsn::resource::Battery &battery) : Component(argc, argv, name), active(active), max_size(5), buffer_size(0), battery(battery), data_buffer() {}

CentralHub::~CentralHub() {}

int32_t CentralHub::run() {

    arch::target_system::Component::setUp();
	setUp();
    std::cout << "setUp();" << std::endl;

    ros::NodeHandle nh;
    ros::Subscriber thermometerSub = nh.subscribe("thermometer_data", 10, &CentralHub::collect, this);
    ros::Subscriber oximeterSub = nh.subscribe("oximeter_data", 10, &CentralHub::collect, this);
    ros::Subscriber ecgSub = nh.subscribe("ecg_data", 10, &CentralHub::collect, this);
    ros::Subscriber diastolicSub = nh.subscribe("diastolic_data", 10, &CentralHub::collect, this);
    ros::Subscriber systolicSub = nh.subscribe("systolic_data", 10, &CentralHub::collect, this);
    std::cout << "ros::NodeHandle nh;" << std::endl;

    sendStatus("init");

    while(ros::ok()) {
        ros::Rate loop_rate(rosComponentDescriptor.getFreq());

        try{
            body();
            std::cout << "body();" << std::endl;
        } catch (const std::exception& e) {
            sendStatus("fail");
        } 
        loop_rate.sleep();
    }
    sendStatus("finish");

    tearDown();
    arch::target_system::Component::tearDown();

    return 0;
}

void CentralHub::body() {
    
    ros::spinOnce(); //calls collect if there's data in the topics

    if (!isActive() && battery.getCurrentLevel() > 90){
        turnOn();
    } else if (isActive() && battery.getCurrentLevel() < 2){
        turnOff();        
    }

    if(isActive()) {
        if(buffer_size > 0){
            sendStatus("running");
            apply_noise();
            std::cout << "    apply_noise();" << std::endl;
            process();
            std::cout << "    body();" << std::endl;
            transfer();
            std::cout << "    transfer();" << std::endl;
            sendStatus("success");
        }
    } else {
        sendStatus("recharging");
        recharge();
    }
}

void CentralHub::apply_noise() {}


void CentralHub::reconfigure(const archlib::AdaptationCommand::ConstPtr& msg) {
    std::string action = msg->action.c_str();

    char *buffer = strdup(action.c_str());
    char *pair = strtok(buffer, ",");
    char *key = strtok(pair, "=");
    double value  = std::stod(strtok(NULL, "="));

    if(key=="freq"){
        rosComponentDescriptor.setFreq(value);
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

/*  battery will always recover in 20seconds
    *
    *  b/s = 100% / 20 seconds = 5 %/s 
    *      => recovers 5% battery per second
    *  if we divide by the execution frequency
    *  we get the amount of battery we need to
    *  recover per execution cycle to achieve the
    *  5 %/s battery recovery rate
    */
void CentralHub::recharge() {
    if(battery.getCurrentLevel() <= 100) 
        battery.generate((100/20)/rosComponentDescriptor.getFreq());
}