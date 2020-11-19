#include <gtest/gtest.h>
#include <ros/ros.h>
#include "../include/SensorTestClass.hpp"

class G3T11Test : public ::testing::Test {
    protected:
        virtual void SetUp() {
            g3t11TestClass = SensorTestClass("g3t1_1", "oximeter", 1.0);
            nh.reset(new ros::NodeHandle);
        }

        virtual void TearDown() {

        }
        SensorTestClass g3t11TestClass;
        std::shared_ptr<ros::NodeHandle> nh;

    G3T11Test() {}
    ~G3T11Test() {}

};

TEST_F(G3T11Test, LowerLimitFrequencyChange) {
    g3t11TestClass.frequencyTestSetup("4.9", nh);
    EXPECT_NEAR(g3t11TestClass.getFreq(), 1.0, 0.1);
}

TEST_F(G3T11Test, UpperLimitFrequencyChange) {
    g3t11TestClass.frequencyTestSetup("25.1", nh);
    EXPECT_NEAR(g3t11TestClass.getFreq(), 1.0, 0.1);
}

TEST_F(G3T11Test, NegativeFrequencyChange) {
    g3t11TestClass.frequencyTestSetup("-1.0", nh);
    EXPECT_NEAR(g3t11TestClass.getFreq(), 1.0, 0.1);
    
}

TEST_F(G3T11Test, ValidFrequencyChange) {
    g3t11TestClass.frequencyTestSetup("5.1", nh);
    EXPECT_NEAR(g3t11TestClass.getFreq(), 5.1, 0.1);
}
