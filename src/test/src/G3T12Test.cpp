#include <gtest/gtest.h>
#include <ros/ros.h>
#include "../include/SensorTestClass.hpp"

class G3T12Test : public ::testing::Test {
    protected:
        virtual void SetUp() {
            g3t12TestClass = SensorTestClass("g3t1_2", "ecg", 1.0);
            nh.reset(new ros::NodeHandle);
        }

        virtual void TearDown() {

        }
        SensorTestClass g3t12TestClass;
        std::shared_ptr<ros::NodeHandle> nh;

    G3T12Test() {}
    ~G3T12Test() {}

};

TEST_F(G3T12Test, LowerLimitFrequencyChange) {
    g3t12TestClass.frequencyTestSetup("4.9", nh);
    EXPECT_NEAR(g3t12TestClass.getFreq(), 1.0, 0.1);
}

TEST_F(G3T12Test, UpperLimitFrequencyChange) {
    g3t12TestClass.frequencyTestSetup("25.1", nh);
    EXPECT_NEAR(g3t12TestClass.getFreq(), 1.0, 0.1);
}

TEST_F(G3T12Test, NegativeFrequencyChange) {
    g3t12TestClass.frequencyTestSetup("-1.0", nh);
    EXPECT_NEAR(g3t12TestClass.getFreq(), 1.0, 0.1);
    
}

TEST_F(G3T12Test, ValidFrequencyChange) {
    g3t12TestClass.frequencyTestSetup("5.1", nh);
    EXPECT_NEAR(g3t12TestClass.getFreq(), 5.1, 0.1);
}
