#include <gtest/gtest.h>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>


//#include "./unit/terget_system/components/component/g4t1/G4T1Test.cpp"
#include "services/PatientData.h"
#include "component/g4t1/G4T1.hpp"
#include "messages/SensorFrequency.h"
#include "archlib/Status.h"
#include "classes/SensorTest.cpp"

bool receivedMessage = false;

SensorTest g3t12TestClass = SensorTest("g3t1_2", "ecg", 1.0);

std::shared_ptr<ros::NodeHandle> nh;

void ecgCallback(const messages::SensorData::ConstPtr& msg)
{
    std::string path = ros::package::getPath("test_suite");
    
    std::ofstream myfile (path + "/test_logs/callback_output.txt");

    if (myfile.is_open()) {

        myfile << msg->type << std::endl;
        myfile << msg->data << std::endl;
        myfile << msg->risk << std::endl;
        myfile << msg->batt << std::endl;
        
        myfile.close();
    }
   // receivedMessage=true;

    EXPECT_EQ(msg->type, "ecg");
    EXPECT_NEAR(msg->data, 95.0201, 0.001);
    EXPECT_NEAR(msg->risk, 13.4003, 0.001);
    EXPECT_NEAR(msg->batt, 98.5, 0.001);

    return;
}

void ecgFreqCallback(const messages::SensorFrequency::ConstPtr& msg) {
    std::string path = ros::package::getPath("test_suite");
    
    std::ofstream myfile (path + "/test_logs/ecg_freq_callback_output.txt");

    if (myfile.is_open()) {
        myfile << msg->type << std::endl;
        myfile << msg->value << std::endl;
        
        myfile.close();
    }

    g3t12TestClass.setFreq(msg->value);

    g3t12TestClass.setReceivedMessage(true);
}


void oxiFreqCallback(const messages::SensorFrequency::ConstPtr& msg) {
    std::string path = ros::package::getPath("test_suite");
    
    std::ofstream myfile (path + "/test_logs/oxi_freq_callback_output.txt");

    if (myfile.is_open()) {
        myfile << msg->type << std::endl;
        myfile << msg->value << std::endl;
        
        myfile.close();
    }

    receivedMessage = true;
}

void statusCallback(const archlib::Status::ConstPtr& msg) {
    std::string path = ros::package::getPath("test_suite");
    
    std::ofstream myfile (path + "/test_logs/status_callback_output.txt");

    if (myfile.is_open()) {
        myfile << "entered callback" << std::endl;
        myfile << msg->source<< std::endl;
        myfile << msg->target << std::endl;
        myfile << msg->content << std::endl;

        myfile.close();
    }

    receivedMessage = true;
}

bool getPatientData(services::PatientData::Request &request, 
                    services::PatientData::Response &response) {
    
    response.data = 95.0201;
    
    return true;
}

/*
TEST(G4T1TestSuite, G4T1Test) {

    ros::NodeHandle nh;
    ros::Publisher therm_pub = nh.advertise<messages::SensorData>("thermometer_data", 10);
    messages::SensorData msg;

    msg.type = "thermometer";
    msg.data = 54.0;
    msg.risk = 1.0;
    msg.batt = 60.0;

    therm_pub.publish(msg);

    while(ros::ok() && receivedMessage == false) {
        ros::Rate loop_rate(100);
        ros::spinOnce();
        loop_rate.sleep();
    }
}


void prepareG3T1_2Test(const SensorTest::ConstPtr& sensor) {
    ros::ServiceServer service = nh->advertiseService("getPatientData", &getPatientData);
    ros::Subscriber ecg_sub = nh->subscribe("/ecg_data", 1, &ecgCallback, sensor);
    ros::Subscriber ecg_freq_sub = nh->subscribe("/sensor_frequency_/g3t1_2", 1, &ecgFreqCallback);
    ros::Publisher ecg_pub = nh->advertise<archlib::AdaptationCommand>("/reconfigure_/g3t1_2", 1);
    //ros::Subscriber ecg_freq_sub = nh->subscribe("/collect_status", 1, &statusCallback);

    ros::Rate loop_rate(100);

    archlib::AdaptationCommand msg;
    msg.source = "/enactor";
    msg.target = "/g3t1_2";
    msg.action = "freq=5.0";

    while (ros::ok() && receivedMessage == false) {
        ecg_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}



TEST(G3T1_1, EffectorFrequencyChange) {

    ros::ServiceServer service = nh->advertiseService("getPatientData", &getPatientData);
//    ros::Subscriber ecg_sub = nh->subscribe("/ecg_data", 1, &oxiCallback);
    ros::Subscriber ecg_freq_sub = nh->subscribe("/sensor_frequency_/g3t1_1", 1, &oxiFreqCallback);
    ros::Publisher ecg_pub = nh->advertise<archlib::AdaptationCommand>("/reconfigure_/g3t1_1", 1);
    //ros::Subscriber ecg_freq_sub = nh->subscribe("/collect_status", 1, &statusCallback);

    ros::Rate loop_rate(100);

    archlib::AdaptationCommand msg;
    msg.source = "/enactor";
    msg.target = "/g3t1_1";
    msg.action = "freq=6.0";

    while (ros::ok() && receivedMessage == false) {
        ecg_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    receivedMessage = false;
}

void sensorFrequencyTestSetup(SensorTest& sensor) {
    //ros::ServiceServer service = nh->advertiseService("getPatientData", &getPatientData);
    ros::Subscriber ecg_sub = nh->subscribe("/" + sensor->getName() +"_data", 1, &ecgCallback);
    ros::Subscriber ecg_freq_sub = nh->subscribe("/sensor_frequency_/" + sensor->getName(), 1, &ecgFreqCallback);
    ros::Publisher ecg_pub = nh->advertise<archlib::AdaptationCommand>("/reconfigure_/g3t1_2", 1);
    //ros::Subscriber ecg_freq_sub = nh->subscribe("/collect_status", 1, &statusCallback);

    ros::Rate loop_rate(100);

    archlib::AdaptationCommand msg;
    msg.source = "/enactor";
    msg.target = "/g3t1_2";
    msg.action = "freq=6.0";

    while (ros::ok() && g3t12TestClass.getReceivedMessage() == false) {
        ecg_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
*/


TEST(G3T1_2, EffectorFrequencyChange) {
    
    g3t12TestClass.frequencyTestSetup("6.0");
    EXPECT_NEAR(g3t12TestClass.getFreq(), 6.0, 0.001);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "TestNode");
    nh.reset(new ros::NodeHandle);
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

