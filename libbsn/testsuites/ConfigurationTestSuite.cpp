#include <gtest/gtest.h>
#include "bsn/configuration/SensorConfiguration.hpp"

using namespace std;
using namespace bsn::range;
using namespace bsn::configuration;

Range l;
Range m1;
Range m2;
Range h1;
Range h2;

array<Range, 2> a1;
array<Range, 2> a2;
array<Range, 3> percentages;

SensorConfiguration s;

class ConfigurationTestSuite : public testing::Test {
    protected:
        void SetUp() override {
            l.setLowerBound(36.5);
            l.setUpperBound(37.5);

            m1.setLowerBound(33);
            m1.setUpperBound(35);

            h1.setLowerBound(20);
            h1.setUpperBound(32);

            m2.setLowerBound(37.6);
            m2.setUpperBound(39.0);

            h2.setLowerBound(39.1);
            h2.setUpperBound(43.0);

            percentages = {Range(0, 0.2), Range(0.21, 0.65), Range(0.66, 1.0)};
            a1 = {m1, m2};
            a2 = {h1, h2};
        }
};

TEST(ConfigurationTest, Constructor) {
    SensorConfiguration s(1, l, a1, a2, percentages);
    
    ASSERT_EQ(1, s.getId());
}

TEST(ConfigurationTest, UnknowValue) {
    ASSERT_EQ(s.evaluateNumber(44), -1);
    ASSERT_EQ(s.evaluateNumber(10), -1);
}

TEST(ConfigurationTest, GetDisplacementNormal) {
    SensorConfiguration s(1, l, a1, a2, percentages);

    ASSERT_EQ(s.getDisplacement(Range(0, 10), 5, "crescent"), 0.5);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 10, "crescent"), 1);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 0, "crescent"), 0);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 7.5, "crescent"), 0.75);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 2.5, "crescent"), 0.25);
}

TEST(ConfigurationTest, GetDisplacementInverse) {
    SensorConfiguration s(1, l, a1, a2, percentages);

    ASSERT_EQ(s.getDisplacement(Range(0, 10), 5, "decrescent"), 0.5);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 10, "decrescent"), 0);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 0, "decrescent"), 1);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 7.5, "decrescent"), 0.25);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 2.5, "decrescent"), 0.75);
}

TEST(ConfigurationTest, GetDisplacementMedium) {
    SensorConfiguration s(1, l, a1, a2, percentages);

    ASSERT_EQ(s.getDisplacement(Range(0, 10), 5, "medium"), 0.0);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 10, "medium"), 1);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 0, "medium"), 1);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 7.5, "medium"), 0.50);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 2.5, "medium"), 0.50);

    ASSERT_EQ(s.getDisplacement(Range(36.5, 37.5), 37.0, "medium"), 0);
    ASSERT_EQ(s.getDisplacement(Range(36.5, 37.5), 36.5, "medium"), 1);
    ASSERT_EQ(s.getDisplacement(Range(36.5, 37.5), 37.5, "medium"), 1);
}

TEST(ConfigurationTest, GetDisplacementInvalidArgument) {
    SensorConfiguration s(1, l, a1, a2, percentages);

    ASSERT_THROW(s.getDisplacement(Range(0, 10), 5, "kjflakj"), std::invalid_argument);
}

TEST(ConfigurationTest, Low) {        
    SensorConfiguration s(1,l,a1,a2,percentages);

    ASSERT_EQ(s.evaluateNumber(37), 0);
    ASSERT_EQ(s.evaluateNumber(37.5), 0.2);
    ASSERT_EQ(s.evaluateNumber(36.5), 0.2);
    ASSERT_EQ(s.evaluateNumber(36.75), 0.1);
}

TEST(ConfigurationTest, Medium0) {
    SensorConfiguration s(1,l,a1,a2,percentages);

    ASSERT_EQ(s.evaluateNumber(33), 0.65);
    ASSERT_EQ(s.evaluateNumber(35), 0.21);
}

TEST(ConfigurationTest, Medium1) {   
    SensorConfiguration s(1,l,a1,a2,percentages);

    ASSERT_EQ(s.evaluateNumber(37.6), 0.21);
    ASSERT_EQ(s.evaluateNumber(39), 0.65);
}

TEST(ConfigurationTest, High0) {
    SensorConfiguration s(1,l,a1,a2,percentages);

    ASSERT_EQ(s.evaluateNumber(20), 1);
    ASSERT_EQ(s.evaluateNumber(32), 0.66);
}

TEST(ConfigurationTest, High1) {            
    SensorConfiguration s(1,l,a1,a2,percentages);

    ASSERT_EQ(s.evaluateNumber(39.1), 0.66);
    ASSERT_EQ(s.evaluateNumber(43), 1.0);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}