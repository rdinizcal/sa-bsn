#include <gtest/gtest.h>
#include <ros/ros.h>
#include "../include/CentralhubTestClass.hpp"

class G4T1Test : public ::testing::Test {
    protected:
        virtual void SetUp() {
            g4t1TestClass = CentralhubTestClass();
            nh.reset(new ros::NodeHandle);
        }

        virtual void TearDown() {

        }
        CentralhubTestClass g4t1TestClass;
        std::shared_ptr<ros::NodeHandle> nh;

    G4T1Test() {}
    ~G4T1Test() {}
};

TEST_F(G4T1Test, TestCentralhubWithExpectedValues) {

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


    g4t1TestClass.setupCentralhubTest(thermMsg, ecgMsg, oxiMsg, abpsMsg, abpdMsg, nh);

    EXPECT_NEAR(g4t1TestClass.getTrmBatt(), thermMsg.batt, 0.0001);
    EXPECT_NEAR(g4t1TestClass.getTrmData(), thermMsg.data, 0.0001);
    EXPECT_NEAR(g4t1TestClass.getTrmRisk(), thermMsg.risk, 0.0001);

    EXPECT_NEAR(g4t1TestClass.getEcgBatt(), ecgMsg.batt, 0.0001);
    EXPECT_NEAR(g4t1TestClass.getEcgData(), ecgMsg.data, 0.0001);
    EXPECT_NEAR(g4t1TestClass.getEcgRisk(), ecgMsg.risk, 0.0001);

    EXPECT_NEAR(g4t1TestClass.getOxiBatt(), oxiMsg.batt, 0.0001);
    EXPECT_NEAR(g4t1TestClass.getOxiData(), oxiMsg.data, 0.0001);
    EXPECT_NEAR(g4t1TestClass.getOxiRisk(), oxiMsg.risk, 0.0001);

    EXPECT_NEAR(g4t1TestClass.getAbpsBatt(), abpsMsg.batt, 0.0001);
    EXPECT_NEAR(g4t1TestClass.getAbpsData(), abpsMsg.data, 0.0001);
    EXPECT_NEAR(g4t1TestClass.getAbpsRisk(), abpsMsg.risk, 0.0001);
    
    EXPECT_NEAR(g4t1TestClass.getAbpdBatt(), abpdMsg.batt, 0.0001);
    EXPECT_NEAR(g4t1TestClass.getAbpdData(), abpdMsg.data, 0.0001);
    EXPECT_NEAR(g4t1TestClass.getAbpdRisk(), abpdMsg.risk, 0.0001);

    EXPECT_NEAR(g4t1TestClass.getPatientStatus(), 68.0, 1.0);
}

TEST_F(G4T1Test, TestCentralhubWithZeroPatientRisk) {
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

    g4t1TestClass.setupCentralhubTest(thermMsg, ecgMsg, oxiMsg, abpsMsg, abpdMsg, nh);

    EXPECT_NEAR(g4t1TestClass.getTrmBatt(), thermMsg.batt, 0.01);
    EXPECT_NEAR(g4t1TestClass.getTrmData(), thermMsg.data, 0.01);
    EXPECT_NEAR(g4t1TestClass.getTrmRisk(), thermMsg.risk, 0.01);

    EXPECT_NEAR(g4t1TestClass.getEcgBatt(), ecgMsg.batt, 0.01);
    EXPECT_NEAR(g4t1TestClass.getEcgData(), ecgMsg.data, 0.01);
    EXPECT_NEAR(g4t1TestClass.getEcgRisk(), ecgMsg.risk, 0.01);

    EXPECT_NEAR(g4t1TestClass.getOxiBatt(), oxiMsg.batt, 0.01);
    EXPECT_NEAR(g4t1TestClass.getOxiData(), oxiMsg.data, 0.01);
    EXPECT_NEAR(g4t1TestClass.getOxiRisk(), oxiMsg.risk, 0.01);

    EXPECT_NEAR(g4t1TestClass.getAbpsBatt(), abpsMsg.batt, 0.01);
    EXPECT_NEAR(g4t1TestClass.getAbpsData(), abpsMsg.data, 0.01);
    EXPECT_NEAR(g4t1TestClass.getAbpsRisk(), abpsMsg.risk, 0.01);
    
    EXPECT_NEAR(g4t1TestClass.getAbpdBatt(), abpdMsg.batt, 0.01);
    EXPECT_NEAR(g4t1TestClass.getAbpdData(), abpdMsg.data, 0.01);
    EXPECT_NEAR(g4t1TestClass.getAbpdRisk(), abpdMsg.risk, 0.01);

    EXPECT_NEAR(g4t1TestClass.getPatientStatus(), 0, 0.01);

}