#include <gtest/gtest.h>
#include "libbsn/filters/MovingAverage.hpp"

using namespace std;
using namespace bsn::filters;

class FiltersTest : public testing::Test {
    protected:
        FiltersTest() {}

        virtual void SetUp() {}
};

TEST_F(FiltersTest, InitialAverage) {
    MovingAverage filter(1);

    ASSERT_EQ(filter.getValue(), 0.0);
}

TEST_F(FiltersTest, AverageWithOneValue) {
    MovingAverage filter(1);

    filter.insert(1);

    ASSERT_EQ(filter.getValue(), 1.0);
}

TEST_F(FiltersTest, AverageWithFiveValues) {
    MovingAverage filter(5);

    filter.insert(32.0);
    filter.insert(35.0);
    filter.insert(36.0);
    filter.insert(34.0);
    filter.insert(35.0);

    ASSERT_EQ(filter.getValue(), 34.4);
}

TEST_F(FiltersTest, AverageWithLessValues) {
    MovingAverage filter(5);

    filter.insert(32.0);
    filter.insert(35.0);
    filter.insert(36.0);
    filter.insert(34.0);

    ASSERT_EQ(filter.getValue(), 34.25);
}

TEST_F(FiltersTest, AverageWithMoreValues) {
    MovingAverage filter(5);

    filter.insert(200);
    filter.insert(32.0);
    filter.insert(35.0);
    filter.insert(36.0);
    filter.insert(34.0);
    filter.insert(35.0);
    filter.insert(37.0);

    ASSERT_EQ(filter.getValue(), 35.4);
}
