#include <gtest/gtest.h>
#include <ros/ros.h>
#include "../include/SensorTestClass.hpp"

class G3T14Test : public ::testing::Test {
    protected:
        virtual void SetUp() {
            g3t14TestClass = SensorTestClass("g3t1_4", "abps", 1.0);
            nh.reset(new ros::NodeHandle);
        }

        virtual void TearDown() {

        }
        SensorTestClass g3t14TestClass;
        std::shared_ptr<ros::NodeHandle> nh;

    G3T14Test() {}
    ~G3T14Test() {}

};

TEST_F(G3T14Test, LowerLimitFrequencyChange) {
    g3t14TestClass.frequencyTestSetup("4.9", nh);
    EXPECT_NEAR(g3t14TestClass.getFreq(), 1.0, 0.1);
}

TEST_F(G3T14Test, UpperLimitFrequencyChange) {
    g3t14TestClass.frequencyTestSetup("25.1", nh);
    EXPECT_NEAR(g3t14TestClass.getFreq(), 1.0, 0.1);
}

TEST_F(G3T14Test, NegativeFrequencyChange) {
    g3t14TestClass.frequencyTestSetup("-1.0", nh);
    EXPECT_NEAR(g3t14TestClass.getFreq(), 1.0, 0.1);
    
}

TEST_F(G3T14Test, ValidFrequencyChange) {
    g3t14TestClass.frequencyTestSetup("5.1", nh);
    EXPECT_NEAR(g3t14TestClass.getFreq(), 5.1, 0.1);
}
