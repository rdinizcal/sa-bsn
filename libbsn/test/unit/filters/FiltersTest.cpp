#include <gtest/gtest.h>
#include "filters/MovingAverage.hpp"

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
        }
};

TEST_F(FiltersTest, InitialAverage) {
    ASSERT_EQ(avg1.getValue(), 0.0);
}

TEST_F(FiltersTest, AverageWithOneValue) {
    avg1.insert(1);

    ASSERT_EQ(avg1.getValue(), 1.0);
}

TEST_F(FiltersTest, AverageWithFiveValues) {
    avg5.insert(32.0);
    avg5.insert(35.0);
    avg5.insert(36.0);
    avg5.insert(34.0);
    avg5.insert(35.0);

    ASSERT_EQ(avg5.getValue(), 34.4);
}

TEST_F(FiltersTest, AverageWithLessValues) {
    avg5.insert(32.0);
    avg5.insert(35.0);
    avg5.insert(36.0);
    avg5.insert(34.0);

    ASSERT_EQ(avg5.getValue(), 34.0);
}

TEST_F(FiltersTest, AverageWithMoreValues) {
    avg5.insert(200);
    avg5.insert(32.0);
    avg5.insert(35.0);
    avg5.insert(36.0);
    avg5.insert(34.0);
    avg5.insert(35.0);
    avg5.insert(37.0);

    ASSERT_EQ(avg5.getValue(), 35.4);
}
