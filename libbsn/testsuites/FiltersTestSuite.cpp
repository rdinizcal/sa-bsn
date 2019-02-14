#include <gtest/gtest.h>
#include "bsn/filters/MovingAverage.hpp"

using namespace std;
using namespace bsn::filters;

class FiltersTest : public testing::Test {
    protected:
        MovingAverage avg1;
        MovingAverage avg2;
        MovingAverage avg5;

        FiltersTest() : avg1(), avg2(), avg5() {}

        virtual void SetUp() {
            avg1.setRange(1);
            avg5.setRange(5);
            avg2.setRange(2);

            // avg1->setRange(1);
        }
};

TEST_F(FiltersTest, InitialAverage) {
    ASSERT_EQ(avg1.getValue("thermometer"), 0.0);
}

TEST_F(FiltersTest, AverageWithOneValue) {
    avg1.insert(1, "thermometer");

    ASSERT_EQ(avg1.getValue("thermometer"), 1.0);
}

TEST_F(FiltersTest, AverageWithFiveValues) {
    avg5.insert(32.0, "thermometer");
    avg5.insert(35.0, "thermometer");
    avg5.insert(36.0, "thermometer");
    avg5.insert(34.0, "thermometer");
    avg5.insert(35.0, "thermometer");

    ASSERT_EQ(avg5.getValue("thermometer"), 34.4);
}

TEST_F(FiltersTest, AverageWithLessValues) {
    avg5.insert(32.0, "thermometer");
    avg5.insert(35.0, "thermometer");
    avg5.insert(36.0, "thermometer");
    avg5.insert(34.0, "thermometer");

    ASSERT_EQ(avg5.getValue("thermometer"), 34.0);
}

TEST_F(FiltersTest, AverageWithMoreValues) {
    avg5.insert(200, "thermometer");
    avg5.insert(32.0, "thermometer");
    avg5.insert(35.0, "thermometer");
    avg5.insert(36.0, "thermometer");
    avg5.insert(34.0, "thermometer");
    avg5.insert(35.0, "thermometer");
    avg5.insert(37.0, "thermometer");

    ASSERT_EQ(avg5.getValue("thermometer"), 35.4);
}

TEST_F(FiltersTest, AverageOfFiveTypesOfSensors) {
    avg5.insert(36.5, "thermometer");
    avg5.insert(80.0, "ecg");
    avg5.insert(100.0, "oximeter");
    avg5.insert(12.0, "bpms");
    avg5.insert(8.0, "bpmd");

    ASSERT_EQ(avg5.getValue("thermometer"), 36.5);
    ASSERT_EQ(avg5.getValue("ecg"), 80.0);
    ASSERT_EQ(avg5.getValue("oximeter"), 100);
    ASSERT_EQ(avg5.getValue("bpms"), 12.0);
    ASSERT_EQ(avg5.getValue("bpmd"), 8.0);
}

TEST_F(FiltersTest, AverageOfFiveTypesOfSensorsWithTwoValues) {
    avg2.insert(36.5, "thermometer");
    avg2.insert(37.5, "thermometer");
    avg2.insert(80.0, "ecg");
    avg2.insert(100.0, "ecg");
    avg2.insert(90.0, "oximeter");
    avg2.insert(100.0, "oximeter");
    avg2.insert(12.0, "bpms");
    avg2.insert(14.0, "bpms");
    avg2.insert(8.0, "bpmd");
    avg2.insert(10.0, "bpmd");

    ASSERT_EQ(avg2.getValue("thermometer"), 37.0);
    ASSERT_EQ(avg2.getValue("ecg"), 90.0);
    ASSERT_EQ(avg2.getValue("oximeter"), 95);
    ASSERT_EQ(avg2.getValue("bpms"), 13.0);
    ASSERT_EQ(avg2.getValue("bpmd"), 9.0);
}