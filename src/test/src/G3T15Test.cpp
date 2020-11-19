#include <gtest/gtest.h>
#include <ros/ros.h>
#include "../include/SensorTestClass.hpp"

class G3T15Test : public ::testing::Test {
    protected:
        virtual void SetUp() {
            g3t15TestClass = SensorTestClass("g3t1_5", "abpd", 1.0);
            nh.reset(new ros::NodeHandle);
        }

        virtual void TearDown() {

        }
        SensorTestClass g3t15TestClass;
        std::shared_ptr<ros::NodeHandle> nh;

    G3T15Test() {}
    ~G3T15Test() {}

};

TEST_F(G3T15Test, LowerLimitFrequencyChange) {
    g3t15TestClass.frequencyTestSetup("4.9", nh);
    EXPECT_NEAR(g3t15TestClass.getFreq(), 1.0, 0.1);
}

TEST_F(G3T15Test, UpperLimitFrequencyChange) {
    g3t15TestClass.frequencyTestSetup("25.1", nh);
    EXPECT_NEAR(g3t15TestClass.getFreq(), 1.0, 0.1);
}

TEST_F(G3T15Test, NegativeFrequencyChange) {
    g3t15TestClass.frequencyTestSetup("-1.0", nh);
    EXPECT_NEAR(g3t15TestClass.getFreq(), 1.0, 0.1);
    
}

TEST_F(G3T15Test, ValidFrequencyChange) {
    g3t15TestClass.frequencyTestSetup("5.1", nh);
    EXPECT_NEAR(g3t15TestClass.getFreq(), 5.1, 0.1);
}
