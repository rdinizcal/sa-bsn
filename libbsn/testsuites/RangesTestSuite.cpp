#include <gtest/gtest.h>
#include <vector>

#include "bsn/range/Range.hpp"

using namespace std;
using namespace bsn::range;

class RangesTest : public testing::Test {
    protected:
        Range r1;
        Range r2;

        RangesTest() : r1(), r2() {}

        virtual void SetUp () {
            r1.setLowerBound(2.5);
            r1.setUpperBound(5.0);

            r2.setLowerBound(0.0);
            r2.setUpperBound(10.0);
        }
};

TEST_F(RangesTest, TestConstructor) {
    ASSERT_EQ(r1.getLowerBound(), 2.5);
    ASSERT_EQ(r1.getUpperBound(), 5.0);
}

TEST_F(RangesTest, IllegalConstruction) {
    ASSERT_THROW(Range r(7.5, 5.0), std::invalid_argument);
}

TEST_F(RangesTest, InRange) {
    ASSERT_EQ(true, r1.in_range(3.0));
}

TEST_F(RangesTest, OutRange) {
    ASSERT_EQ(false, r1.in_range(8.0));
}

TEST_F(RangesTest, Convert) {
    ASSERT_EQ(3.0, r2.convert(2, 3, 10.0));
}