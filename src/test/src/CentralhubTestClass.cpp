#include "../include/CentralhubTestClass.hpp"


CentralhubTestClass::CentralhubTestClass() {}
CentralhubTestClass::~CentralhubTestClass() {}

void CentralhubTestClass::processTargetSystemData(const messages::TargetSystemData::ConstPtr &msg) {
/*
    std::string path = ros::package::getPath("test_suite");
    
    std::ofstream myfile (path + "/test_logs/g4t1_output.txt");

    if (myfile.is_open()) {
        
        myfile << msg->trm_batt<< std::endl;
        myfile << msg->ecg_batt<< std::endl;
        myfile << msg->oxi_batt<< std::endl;
        myfile << msg->abps_batt<< std::endl;
        myfile << msg->abpd_batt<< std::endl;
        myfile << msg->trm_risk<< std::endl;
        myfile << msg->ecg_risk<< std::endl;
        myfile << msg->oxi_risk<< std::endl;
        myfile << msg->abps_risk<< std::endl;
        myfile << msg->abpd_risk<< std::endl;
        myfile << msg->trm_data<< std::endl;
        myfile << msg->ecg_data<< std::endl;
        myfile << msg->oxi_data<< std::endl;
        myfile << msg->abps_data<< std::endl;
        myfile << msg->abpd_data<< std::endl;
        myfile << msg->patient_status<< std::endl;

        myfile.close();
    }
*/
        setTrmBatt(msg->trm_batt);
        setEcgBatt(msg->ecg_batt);
        setOxiBatt(msg->oxi_batt);
        setAbpsBatt(msg->abps_batt);
        setAbpdBatt(msg->abpd_batt);
        setTrmRisk(msg->trm_risk);
        setEcgRisk(msg->ecg_risk);
        setOxiRisk(msg->oxi_risk);
        setAbpsRisk(msg->abps_risk);
        setAbpdRisk(msg->abpd_risk);
        setTrmData(msg->trm_data);
        setEcgData(msg->ecg_data);
        setOxiData(msg->oxi_data);
        setAbpsData(msg->abps_data);
        setAbpdData(msg->abpd_data);
        setPatientStatus(msg->patient_status);

}

void CentralhubTestClass::setupCentralhubTest(messages::SensorData thermMsg,
    messages::SensorData ecgMsg,
    messages::SensorData oxiMsg,
    messages::SensorData abpsMsg,
    messages::SensorData abpdMsg,
    std::shared_ptr<ros::NodeHandle> nh
) {

    ros::Publisher thermometerPub = nh->advertise<messages::SensorData>("thermometer_data", 1);
    ros::Publisher oximeterPub = nh->advertise<messages::SensorData>("oximeter_data", 1);
    ros::Publisher ecgPub = nh->advertise<messages::SensorData>("ecg_data", 1);
    ros::Publisher abpsPub = nh->advertise<messages::SensorData>("abps_data", 1);
    ros::Publisher abpdPub = nh->advertise<messages::SensorData>("abpd_data", 1);

    ros::Subscriber targetSystemSub = nh->subscribe("/TargetSystemData", 1, &CentralhubTestClass::processTargetSystemData, this);

    ros::Rate loop_rate(2.0);
/*
    setTherm(thermMsg);
    setEcg(oxiMsg);
    setOxi(ecgMsg);
    setAbps(abpsMsg);
    setAbpd(abpdMsg);
*/
    int tries = 0;

    while (ros::ok() && tries < 10) {
        thermometerPub.publish(thermMsg);
        oximeterPub.publish(oxiMsg);
        ecgPub.publish(ecgMsg);
        abpsPub.publish(abpsMsg);
        abpdPub.publish(abpdMsg);
        ros::spinOnce();
        tries++;
        loop_rate.sleep();
    }
}

double CentralhubTestClass::getPatientRisk() {return this->patientRisk;}
void CentralhubTestClass::setPatientRisk(double patientRisk) {this->patientRisk = patientRisk;}
double CentralhubTestClass::getTrmBatt() {return this->trmBatt;}
void CentralhubTestClass::setTrmBatt(double trmBatt) {this->trmBatt = trmBatt;}
double CentralhubTestClass::getEcgBatt() {return this->ecgBatt;}
void CentralhubTestClass::setEcgBatt(double ecgBatt) {this->ecgBatt = ecgBatt;}
double CentralhubTestClass::getOxiBatt() {return this->oxiBatt;}
void CentralhubTestClass::setOxiBatt(double oxiBatt) {this->oxiBatt = oxiBatt;}
double CentralhubTestClass::getAbpsBatt() {return this->abpsBatt;}
void CentralhubTestClass::setAbpsBatt(double abpsBatt) {this->abpsBatt = abpsBatt;}
double CentralhubTestClass::getAbpdBatt() {return this->abpdBatt;}
void CentralhubTestClass::setAbpdBatt(double abpdBatt) {this->abpdBatt = abpdBatt;}
double CentralhubTestClass::getTrmRisk() {return this->trmRisk;}
void CentralhubTestClass::setTrmRisk(double trmRisk) {this->trmRisk = trmRisk;}
double CentralhubTestClass::getEcgRisk() {return this->ecgRisk;}
void CentralhubTestClass::setEcgRisk(double ecgRisk) {this->ecgRisk = ecgRisk;}
double CentralhubTestClass::getOxiRisk() {return this->oxiRisk;}
void CentralhubTestClass::setOxiRisk(double oxiRisk) {this->oxiRisk = oxiRisk;}
double CentralhubTestClass::getAbpsRisk() {return this->abpsRisk;}
void CentralhubTestClass::setAbpsRisk(double abpsRisk) {this->abpsRisk = abpsRisk;}
double CentralhubTestClass::getAbpdRisk() {return this->abpdRisk;}
void CentralhubTestClass::setAbpdRisk(double abpdRisk) {this->abpdRisk = abpdRisk;}
double CentralhubTestClass::getTrmData() {return this->trmData;}
void CentralhubTestClass::setTrmData(double trmData) {this->trmData = trmData;}
double CentralhubTestClass::getEcgData() {return this->ecgData;}
void CentralhubTestClass::setEcgData(double ecgData) {this->ecgData = ecgData;}
double CentralhubTestClass::getOxiData() {return this->oxiData;}
void CentralhubTestClass::setOxiData(double oxiData) {this->oxiData = oxiData;}
double CentralhubTestClass::getAbpsData() {return this->abpsData;}
void CentralhubTestClass::setAbpsData(double abpsData) {this->abpsData = abpsData;}
double CentralhubTestClass::getAbpdData() {return this->abpdData;}
void CentralhubTestClass::setAbpdData(double abpdData) {this->abpdData = abpdData;}
double CentralhubTestClass::getPatientStatus() {return this->patientStatus;}
void CentralhubTestClass::setPatientStatus(double patientStatus) {this->patientStatus = patientStatus;}
