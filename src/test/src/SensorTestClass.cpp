#include "../include/SensorTestClass.hpp"

SensorTestClass::SensorTestClass(std::string name, std::string alias, double freqVal) {
    this->name = name;
    this->alias = alias;
    this->freqVal = freqVal;
    this->receivedMessage = false;
}
SensorTestClass::SensorTestClass() {}
SensorTestClass::~SensorTestClass() {}
/*
bool SensorTestClass::getPatientData(services::PatientData::Request &request, 
                    services::PatientData::Response &response) {
    
    response.data = 95.0201;
    
    return true;
}
*/
double SensorTestClass::getFreq() {return this->freqVal;}
void SensorTestClass::setFreq(double freqVal) {this->freqVal = freqVal;}
std::string SensorTestClass::getName() {return this->name;}
std::string SensorTestClass::getAlias() {return this->alias;}
bool SensorTestClass::getReceivedMessage() {return this->receivedMessage;}
void SensorTestClass::setReceivedMessage(bool receivedMessage) {this->receivedMessage = receivedMessage;}

void SensorTestClass::freqCallback(const messages::SensorFrequency::ConstPtr& msg) {
    
    std::string path = ros::package::getPath("test_suite");
    std::ofstream myfile (path + "/test_logs/"+this->getName()+"_freq_callback_output.txt");

    if (myfile.is_open()) {
        myfile << msg->type << std::endl;
        myfile << msg->value << std::endl;
        
        myfile.close();
    }
    
    this->setFreq(msg->value);
    this->setReceivedMessage(true);
}

void SensorTestClass::frequencyTestSetup(std::string val, std::shared_ptr<ros::NodeHandle> nh) {
    //ros::ServiceServer service = nh->advertiseService("getPatientData", &SensorTestClass::getPatientData);
    //ros::Subscriber ecg_sub = nh->subscribe("/" + this->getAlias() +"_data", 1, &SensorTestClass::dataCallback);

    //ros::NodeHandle nh;
    ros::Subscriber freq_sub = nh->subscribe("/sensor_frequency_/" + this->getName(), 1, &SensorTestClass::freqCallback, this);
    ros::Publisher freq_pub = nh->advertise<archlib::AdaptationCommand>("/reconfigure_/" + this->getName(), 1);
    //ros::Subscriber ecg_freq_sub = nh->subscribe("/collect_status", 1, &statusCallback);

    ros::Rate loop_rate(5.0);

    archlib::AdaptationCommand msg;
    msg.source = "/enactor";
    msg.target = "/" + this->name;
    msg.action = "freq=" + val;

    while (ros::ok() && this->getReceivedMessage() == false) {
        freq_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    this->setReceivedMessage(false);
}
