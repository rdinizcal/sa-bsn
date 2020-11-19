#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/package.h>

#include "services/PatientData.h"
#include "messages/SensorFrequency.h"
#include "messages/TargetSystemData.h"
#include "archlib/Status.h"

std::shared_ptr<ros::NodeHandle> nh;
/*
bool getPatientData(services::PatientData::Request &request, 
                    services::PatientData::Response &response) {
    
    response.data = 95.0201;
    
    return true;
}
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "TestNode");
    nh.reset(new ros::NodeHandle);
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
