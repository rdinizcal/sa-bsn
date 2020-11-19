#include <gtest/gtest.h>
#include <ros/ros.h>
#include "../include/SensorTestClass.hpp"

class G3T13Test : public ::testing::Test {
    protected:
        virtual void SetUp() {
            g3t13TestClass = SensorTestClass("g3t1_3", "thermometer", 1.0);
            nh.reset(new ros::NodeHandle);
        }

        virtual void TearDown() {

        }
        SensorTestClass g3t13TestClass;
        std::shared_ptr<ros::NodeHandle> nh;

    G3T13Test() {}
    ~G3T13Test() {}

};

TEST_F(G3T13Test, LowerLimitFrequencyChange) {
    g3t13TestClass.frequencyTestSetup("4.9", nh);
    EXPECT_NEAR(g3t13TestClass.getFreq(), 1.0, 0.1);
}

TEST_F(G3T13Test, UpperLimitFrequencyChange) {
    g3t13TestClass.frequencyTestSetup("25.1", nh);
    EXPECT_NEAR(g3t13TestClass.getFreq(), 1.0, 0.1);
}

TEST_F(G3T13Test, NegativeFrequencyChange) {
    g3t13TestClass.frequencyTestSetup("-1.0", nh);
    EXPECT_NEAR(g3t13TestClass.getFreq(), 1.0, 0.1);
    
}

TEST_F(G3T13Test, ValidFrequencyChange) {
    g3t13TestClass.frequencyTestSetup("5.1", nh);
    EXPECT_NEAR(g3t13TestClass.getFreq(), 5.1, 0.1);
}
