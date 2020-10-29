#include <gtest/gtest.h>
#include <iostream>

#include "ros/ros.h"
#include <ros/package.h>


//#include "./unit/terget_system/components/component/g4t1/G4T1Test.cpp"

#include "component/g4t1/G4T1.hpp"

bool receivedMessage = false;

std::shared_ptr<ros::NodeHandle> nh;

void thermometerCallback(const messages::SensorData::ConstPtr& sensor_data)
{
    std::string path = ros::package::getPath("test_suite");
    
    std::ofstream myfile (path + "/test_logs/callback_output.txt");

    if (myfile.is_open()) {

        myfile << sensor_data->type << std::endl;
        myfile << sensor_data->data << std::endl;
        myfile << sensor_data->risk << std::endl;
        myfile << sensor_data->batt << std::endl;
        
        myfile.close();
    }
    receivedMessage=true;
    return;
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
*/
TEST(TestSuite, G3T1_3Test) {
    
    ros::Subscriber therm_sub = nh->subscribe("/thermometer_data", 1, &thermometerCallback);

    std::string path = ros::package::getPath("test_suite");

    std::ofstream  myfile (path + "/test_logs/test_a.txt");
    ros::Rate loop_rate(100);

    if (myfile.is_open()) {

        myfile << "entered loop";

        while (ros::ok() && receivedMessage == false) {
            ros::spinOnce();
            loop_rate.sleep();
        }
        
        myfile.close();
    }

    EXPECT_EQ(0,0);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "G4T1Test");
    nh.reset(new ros::NodeHandle);
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

