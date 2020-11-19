#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/package.h>

#include "services/PatientData.h"
#include "messages/SensorFrequency.h"
#include "messages/TargetSystemData.h"
#include "archlib/Status.h"

#include "SensorTestClass.hpp"
#include "CentralhubTestClass.hpp"

SensorTestClass g3t11TestClass = SensorTestClass("g3t1_1", "oximeter", 1.0);
SensorTestClass g3t12TestClass = SensorTestClass("g3t1_2", "ecg", 1.0);
SensorTestClass g3t13TestClass = SensorTestClass("g3t1_3", "thermometer", 1.0);
SensorTestClass g3t14TestClass = SensorTestClass("g3t1_4", "abps", 1.0);
SensorTestClass g3t15TestClass = SensorTestClass("g3t1_5", "abpd", 1.0);

CentralhubTestClass g4t1Test = CentralhubTestClass();

std::shared_ptr<ros::NodeHandle> nh;

bool getPatientData(services::PatientData::Request &request, 
                    services::PatientData::Response &response) {
    
    response.data = 95.0201;
    
    return true;
}


TEST(G4T1, TestCentralhubWithExpectedValues) {

    messages::SensorData thermMsg;
    messages::SensorData oxiMsg;
    messages::SensorData ecgMsg;
    messages::SensorData abpsMsg;
    messages::SensorData abpdMsg;


    thermMsg.type = "thermometer";
    thermMsg.batt = 96.000000000000227;
    thermMsg.data = 48.589746857029397;
    thermMsg.risk = 76.399265812300001;
    
    ecgMsg.type = "ecg";
    ecgMsg.batt = 76.900000000000219;
    ecgMsg.data = 98.73879563645832;
    ecgMsg.risk = 25.250389333564783;

    oxiMsg.type = "oximeter";
    oxiMsg.batt = 83.599999999999767;
    oxiMsg.data = 74.280214469614151;
    oxiMsg.risk = 52.444704222465155;

    abpsMsg.type = "abps";
    abpsMsg.batt = 96.600000000000193;
    abpsMsg.data = 135.05796508002251;
    abpsMsg.risk = 69.686328717409538;

    abpdMsg.type = "abpd";
    abpdMsg.batt = 96.900000000000176;
    abpdMsg.data = 62.945620035433457;
    abpdMsg.risk = 69.426413125646604;


    g4t1Test.setupCentralhubTest(thermMsg, ecgMsg, oxiMsg, abpsMsg, abpdMsg, nh);

    EXPECT_NEAR(g4t1Test.getTrmBatt(), thermMsg.batt, 0.0001);
    EXPECT_NEAR(g4t1Test.getTrmData(), thermMsg.data, 0.0001);
    EXPECT_NEAR(g4t1Test.getTrmRisk(), thermMsg.risk, 0.0001);

    EXPECT_NEAR(g4t1Test.getEcgBatt(), ecgMsg.batt, 0.0001);
    EXPECT_NEAR(g4t1Test.getEcgData(), ecgMsg.data, 0.0001);
    EXPECT_NEAR(g4t1Test.getEcgRisk(), ecgMsg.risk, 0.0001);

    EXPECT_NEAR(g4t1Test.getOxiBatt(), oxiMsg.batt, 0.0001);
    EXPECT_NEAR(g4t1Test.getOxiData(), oxiMsg.data, 0.0001);
    EXPECT_NEAR(g4t1Test.getOxiRisk(), oxiMsg.risk, 0.0001);

    EXPECT_NEAR(g4t1Test.getAbpsBatt(), abpsMsg.batt, 0.0001);
    EXPECT_NEAR(g4t1Test.getAbpsData(), abpsMsg.data, 0.0001);
    EXPECT_NEAR(g4t1Test.getAbpsRisk(), abpsMsg.risk, 0.0001);
    
    EXPECT_NEAR(g4t1Test.getAbpdBatt(), abpdMsg.batt, 0.0001);
    EXPECT_NEAR(g4t1Test.getAbpdData(), abpdMsg.data, 0.0001);
    EXPECT_NEAR(g4t1Test.getAbpdRisk(), abpdMsg.risk, 0.0001);

    EXPECT_NEAR(g4t1Test.getPatientStatus(), 68.0, 1.0);
}

TEST(G4T1, TestCentralhubWithZeroPatientRisk) {
    messages::SensorData thermMsg;
    messages::SensorData oxiMsg;
    messages::SensorData ecgMsg;
    messages::SensorData abpsMsg;
    messages::SensorData abpdMsg;

    abpdMsg.type = "abpd";
    abpdMsg.batt = 6.9525147289986856e-310;
    abpdMsg.data = 2.1220281695573725e-314;
    abpdMsg.risk = 0;

    abpsMsg.type = "abps";
    abpsMsg.batt = 2.4703282292062327e-323;
    abpsMsg.data = 0;
    abpsMsg.risk = 0;

    ecgMsg.type = "ecg";
    ecgMsg.batt = 1.4821969375237396e-323;
    ecgMsg.data = 4.648549688463257e-310;
    ecgMsg.risk = 0;

    oxiMsg.type = "oximeter";
    oxiMsg.batt = 98.999999999999986;
    oxiMsg.data = 6.9532680956194044e-310;
    oxiMsg.risk = 0;

    thermMsg.type = "thermometer";
    thermMsg.batt = 4.64854968846217e-310;
    thermMsg.data = 2.9643938750474793e-323;
    thermMsg.risk = 0;

    g4t1Test.setupCentralhubTest(thermMsg, ecgMsg, oxiMsg, abpsMsg, abpdMsg, nh);

    EXPECT_NEAR(g4t1Test.getTrmBatt(), thermMsg.batt, 0.01);
    EXPECT_NEAR(g4t1Test.getTrmData(), thermMsg.data, 0.01);
    EXPECT_NEAR(g4t1Test.getTrmRisk(), thermMsg.risk, 0.01);

    EXPECT_NEAR(g4t1Test.getEcgBatt(), ecgMsg.batt, 0.01);
    EXPECT_NEAR(g4t1Test.getEcgData(), ecgMsg.data, 0.01);
    EXPECT_NEAR(g4t1Test.getEcgRisk(), ecgMsg.risk, 0.01);

    EXPECT_NEAR(g4t1Test.getOxiBatt(), oxiMsg.batt, 0.01);
    EXPECT_NEAR(g4t1Test.getOxiData(), oxiMsg.data, 0.01);
    EXPECT_NEAR(g4t1Test.getOxiRisk(), oxiMsg.risk, 0.01);

    EXPECT_NEAR(g4t1Test.getAbpsBatt(), abpsMsg.batt, 0.01);
    EXPECT_NEAR(g4t1Test.getAbpsData(), abpsMsg.data, 0.01);
    EXPECT_NEAR(g4t1Test.getAbpsRisk(), abpsMsg.risk, 0.01);
    
    EXPECT_NEAR(g4t1Test.getAbpdBatt(), abpdMsg.batt, 0.01);
    EXPECT_NEAR(g4t1Test.getAbpdData(), abpdMsg.data, 0.01);
    EXPECT_NEAR(g4t1Test.getAbpdRisk(), abpdMsg.risk, 0.01);

    EXPECT_NEAR(g4t1Test.getPatientStatus(), 0, 0.01);

}

TEST(G3T1_1, LowerLimitFrequencyChange) {
    g3t11TestClass.frequencyTestSetup("4.9", nh);
    EXPECT_NEAR(g3t11TestClass.getFreq(), 1.0, 0.1);
}

TEST(G3T1_1, UpperLimitFrequencyChange) {
    g3t11TestClass.frequencyTestSetup("25.1", nh);
    EXPECT_NEAR(g3t11TestClass.getFreq(), 1.0, 0.1);
}

TEST(G3T1_1, NegativeFrequencyChange) {
    g3t11TestClass.frequencyTestSetup("-1.0", nh);
    EXPECT_NEAR(g3t11TestClass.getFreq(), 1.0, 0.1);
    
}

TEST(G3T1_1, ValidFrequencyChange) {
    g3t11TestClass.frequencyTestSetup("5.1", nh);
    EXPECT_NEAR(g3t11TestClass.getFreq(), 5.1, 0.1);
}

TEST(G3T1_2, LowerLimitFrequencyChange) {
    g3t12TestClass.frequencyTestSetup("4.9", nh);
    EXPECT_NEAR(g3t12TestClass.getFreq(), 1.0, 0.1);
}

TEST(G3T1_2, UpperLimitFrequencyChange) {
    g3t12TestClass.frequencyTestSetup("25.1", nh);
    EXPECT_NEAR(g3t12TestClass.getFreq(), 1.0, 0.1);
}

TEST(G3T1_2, NegativeFrequencyChange) {
    g3t12TestClass.frequencyTestSetup("-1.0", nh);
    EXPECT_NEAR(g3t12TestClass.getFreq(), 1.0, 0.1);
}

TEST(G3T1_2, ValidFrequencyChange) {
    g3t12TestClass.frequencyTestSetup("5.1", nh);
    EXPECT_NEAR(g3t12TestClass.getFreq(), 5.1, 0.1);
}

TEST(G3T1_3, LowerLimitFrequencyChange) {
    g3t13TestClass.frequencyTestSetup("4.9", nh);
    EXPECT_NEAR(g3t13TestClass.getFreq(), 1.0, 0.1);
}

TEST(G3T1_3, UpperLimitFrequencyChange) {
    g3t13TestClass.frequencyTestSetup("25.1", nh);
    EXPECT_NEAR(g3t13TestClass.getFreq(), 1.0, 0.1);
}

TEST(G3T1_3, NegativeFrequencyChange) {
    g3t13TestClass.frequencyTestSetup("-1.0", nh);
    EXPECT_NEAR(g3t13TestClass.getFreq(), 1.0, 0.1);
}

TEST(G3T1_3, ValidFrequencyChange) {
    g3t13TestClass.frequencyTestSetup("5.1", nh);
    EXPECT_NEAR(g3t13TestClass.getFreq(), 5.1, 0.1);
}

TEST(G3T1_4, LowerLimitFrequencyChange) {
    g3t14TestClass.frequencyTestSetup("4.9", nh);
    EXPECT_NEAR(g3t14TestClass.getFreq(), 1.0, 0.1);
}

TEST(G3T1_4, UpperLimitFrequencyChange) {
    g3t14TestClass.frequencyTestSetup("25.1", nh);
    EXPECT_NEAR(g3t14TestClass.getFreq(), 1.0, 0.1);
}

TEST(G3T1_4, NegativeFrequencyChange) {
    g3t14TestClass.frequencyTestSetup("-1.0", nh);
    EXPECT_NEAR(g3t14TestClass.getFreq(), 1.0, 0.1);
}

TEST(G3T1_4, ValidFrequencyChange) {
    g3t14TestClass.frequencyTestSetup("5.1", nh);
    EXPECT_NEAR(g3t14TestClass.getFreq(), 5.1, 0.1);
}

TEST(G3T1_5, LowerLimitFrequencyChange) {
    g3t15TestClass.frequencyTestSetup("4.9", nh);
    EXPECT_NEAR(g3t15TestClass.getFreq(), 1.0, 0.1);
}

TEST(G3T1_5, UpperLimitFrequencyChange) {
    g3t15TestClass.frequencyTestSetup("25.1", nh);
    EXPECT_NEAR(g3t15TestClass.getFreq(), 1.0, 0.1);
}

TEST(G3T1_5, NegativeFrequencyChange) {
    g3t15TestClass.frequencyTestSetup("-1.0", nh);
    EXPECT_NEAR(g3t15TestClass.getFreq(), 1.0, 0.1);    
}

TEST(G3T1_5, ValidFrequencyChange) {
    g3t15TestClass.frequencyTestSetup("5.1", nh);
    EXPECT_NEAR(g3t15TestClass.getFreq(), 5.1, 0.1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TestNode");
    nh.reset(new ros::NodeHandle);
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
