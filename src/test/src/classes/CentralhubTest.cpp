#include "include/CentralhubTest.hpp"


CentralhubTest::CentralhubTest() {}
CentralhubTest::~CentralhubTest() {}

/*
void CentralhubTest::setTherm(messages::SensorData msg){
    this->therm_batt = msg.therm_batt;
    this->therm_risk = msg.therm_risk;
    this->therm_data = msg.therm_data;
}
void CentralhubTest::setEcg(messages::SensorData msg){
    this->ecg_batt = msg.ecg_batt;
    this->ecg_risk = msg.ecg_risk;
    this->ecg_data = msg.ecg_data;
}
void CentralhubTest::setOxi(messages::SensorData msg){
    this->oxi_batt = msg.oxi_batt;
    this->oxi_risk = msg.oxi_risk;
    this->oxi_data = msg.oxi_data;
}
void CentralhubTest::setAbps(messages::SensorData msg){
    this->abps_batt = msg.abps_batt;
    this->abps_risk = msg.abps_risk;
    this->abps_data = msg.abps_data;
}
void CentralhubTest::setAbpd(messages::SensorData msg) {
    this->abpd_batt = msg.abpd_batt;
    this->abpd_risk = msg.abpd_risk;
    this->abpd_data = msg.abpd_data;
}
*/


void CentralhubTest::processTargetSystemData(const messages::TargetSystemData::ConstPtr &msg) {
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

void CentralhubTest::setupCentralhubTest(messages::SensorData thermMsg,
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

    ros::Subscriber targetSystemSub = nh->subscribe("/TargetSystemData", 1, &CentralhubTest::processTargetSystemData, this);

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

double CentralhubTest::getPatientRisk() {return this->patientRisk;}
void CentralhubTest::setPatientRisk(double patientRisk) {this->patientRisk = patientRisk;}
double CentralhubTest::getTrmBatt() {return this->trmBatt;}
void CentralhubTest::setTrmBatt(double trmBatt) {this->trmBatt = trmBatt;}
double CentralhubTest::getEcgBatt() {return this->ecgBatt;}
void CentralhubTest::setEcgBatt(double ecgBatt) {this->ecgBatt = ecgBatt;}
double CentralhubTest::getOxiBatt() {return this->oxiBatt;}
void CentralhubTest::setOxiBatt(double oxiBatt) {this->oxiBatt = oxiBatt;}
double CentralhubTest::getAbpsBatt() {return this->abpsBatt;}
void CentralhubTest::setAbpsBatt(double abpsBatt) {this->abpsBatt = abpsBatt;}
double CentralhubTest::getAbpdBatt() {return this->abpdBatt;}
void CentralhubTest::setAbpdBatt(double abpdBatt) {this->abpdBatt = abpdBatt;}
double CentralhubTest::getTrmRisk() {return this->trmRisk;}
void CentralhubTest::setTrmRisk(double trmRisk) {this->trmRisk = trmRisk;}
double CentralhubTest::getEcgRisk() {return this->ecgRisk;}
void CentralhubTest::setEcgRisk(double ecgRisk) {this->ecgRisk = ecgRisk;}
double CentralhubTest::getOxiRisk() {return this->oxiRisk;}
void CentralhubTest::setOxiRisk(double oxiRisk) {this->oxiRisk = oxiRisk;}
double CentralhubTest::getAbpsRisk() {return this->abpsRisk;}
void CentralhubTest::setAbpsRisk(double abpsRisk) {this->abpsRisk = abpsRisk;}
double CentralhubTest::getAbpdRisk() {return this->abpdRisk;}
void CentralhubTest::setAbpdRisk(double abpdRisk) {this->abpdRisk = abpdRisk;}
double CentralhubTest::getTrmData() {return this->trmData;}
void CentralhubTest::setTrmData(double trmData) {this->trmData = trmData;}
double CentralhubTest::getEcgData() {return this->ecgData;}
void CentralhubTest::setEcgData(double ecgData) {this->ecgData = ecgData;}
double CentralhubTest::getOxiData() {return this->oxiData;}
void CentralhubTest::setOxiData(double oxiData) {this->oxiData = oxiData;}
double CentralhubTest::getAbpsData() {return this->abpsData;}
void CentralhubTest::setAbpsData(double abpsData) {this->abpsData = abpsData;}
double CentralhubTest::getAbpdData() {return this->abpdData;}
void CentralhubTest::setAbpdData(double abpdData) {this->abpdData = abpdData;}
double CentralhubTest::getPatientStatus() {return this->patientStatus;}
void CentralhubTest::setPatientStatus(double patientStatus) {this->patientStatus = patientStatus;}
