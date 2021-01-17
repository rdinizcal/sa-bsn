#include "component/CentralHub.hpp"

CentralHub::CentralHub(int &argc, char **argv, const std::string &name, const bool &active, const bsn::resource::Battery &battery) : Component(argc, argv, name), active(active), max_size(20), total_buffer_size(0), buffer_size({0,0,0,0,0}), battery(battery), data_buffer({{0},{0},{0},{0},{0}}) {}

CentralHub::~CentralHub() {}

int32_t CentralHub::run() {
	setUp();

    ros::NodeHandle nh;
    ros::Subscriber thermometerSub = nh.subscribe("thermometer_data", 10, &CentralHub::collect, this);
    ros::Subscriber oximeterSub = nh.subscribe("oximeter_data", 10, &CentralHub::collect, this);
    ros::Subscriber ecgSub = nh.subscribe("ecg_data", 10, &CentralHub::collect, this);
    ros::Subscriber abpsSub = nh.subscribe("abps_data", 10, &CentralHub::collect, this);
    ros::Subscriber abpdSub = nh.subscribe("abpd_data", 10, &CentralHub::collect, this);

    statusPub = nh.advertise<messages::CentralhubDiagnostics>("centralhub_diagnostics", 100);

    messages::CentralhubDiagnostics msg;
    msg.id = 0;
    msg.type = "centralhub";
    msg.source = "centralhub";
    msg.status = "on";

    statusPub.publish(msg);

    std::string path = ros::package::getPath("diagnostics_analyzer");
    nh.getParam("property", foldername);

    std::string pathSuffix = "";
    handle.getParam("noise", pathSuffix);

    if (pathSuffix == "true") pathSuffix = "_" + pathSuffix;

    filepath = path + "/../logs/"+foldername+"/centralhub/centralhub"+pathSuffix+".log";

    fp.open(filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "\n";
    fp.close();

    while(ros::ok()) {
        ros::Rate loop_rate(rosComponentDescriptor.getFreq());

        try {
            body();
        } catch (const std::exception& e) {
            sendStatus("fail");
        }
        loop_rate.sleep();
    }

    return 0;
}

void CentralHub::flushData(messages::CentralhubDiagnostics msg) {
    fp.open(filepath, std::fstream::in | std::fstream::out | std::fstream::app);
    fp << msg.timestamp << ",";
    fp << msg.id << ",";
    fp << msg.source << ",";
    fp << msg.status << std::endl;
    fp.close();
}


void CentralHub::body() {
    ros::spinOnce(); //calls collect() if there's data in the topics

    messages::CentralhubDiagnostics msg;
    int32_t dataId;
    boost::posix_time::ptime my_posix_time;

    if (!isActive() && battery.getCurrentLevel() > 90){
        turnOn();
        
        msg.id = 0;
        msg.source = "centralhub";
        msg.status = "on";
        my_posix_time = ros::Time::now().toBoost();
        timestamp = boost::posix_time::to_iso_extended_string(my_posix_time);
        statusPub.publish(msg);
        
        flushData(msg);
    } else if (isActive() && battery.getCurrentLevel() < 2){
        msg.id = 0;
        msg.type = "centralhub";
        msg.status = "off";
        my_posix_time = ros::Time::now().toBoost();
        timestamp = boost::posix_time::to_iso_extended_string(my_posix_time);
        statusPub.publish(msg);
        
        flushData(msg);
        
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

    std::vector<std::string> pairs = bsn::utils::split(action, ',');

    for (std::vector<std::string>::iterator it = pairs.begin(); it != pairs.end(); ++it){
        std::vector<std::string> param = bsn::utils::split(action, '=');

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
        battery.generate((100.0/20.0)/rosComponentDescriptor.getFreq());
}