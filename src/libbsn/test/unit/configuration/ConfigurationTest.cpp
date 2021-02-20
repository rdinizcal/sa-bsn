#include <gtest/gtest.h>
#include "libbsn/configuration/SensorConfiguration.hpp"

using namespace std;
using namespace bsn::range;
using namespace bsn::configuration;


class ConfigurationTest : public testing::Test {
    protected:
        Range l;
        Range m1;
        Range m2;
        Range h1;
        Range h2;

        array<Range, 2> a1;
        array<Range, 2> a2;

        SensorConfiguration s;

        ConfigurationTest() : l(), m1(), m2(), h1(), h2(), a1(), a2(), s() {}

        virtual void SetUp() {
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

            a1 = {m1, m2};
            a2 = {h1, h2};

            s.setId(1);
            s.setLowRisk(l);
            s.setMediumRisk(a1);
            s.setHighRisk(a2);
            s.setLowPercentage(Range(0, 0.2));
            s.setMidPercentage(Range(0.21, 0.65));
            s.setHighPercentage(Range(0.66, 1.0));
        }
};

TEST_F(ConfigurationTest, Constructor) {
    ASSERT_EQ(1, s.getId());
}

TEST_F(ConfigurationTest, UnknowValue) {
    ASSERT_EQ(s.evaluateNumber(44), -1);
    ASSERT_EQ(s.evaluateNumber(10), -1);
}

TEST_F(ConfigurationTest, GetDisplacementNormal) {
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 5, "crescent"), 0.5);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 10, "crescent"), 1);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 0, "crescent"), 0);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 7.5, "crescent"), 0.75);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 2.5, "crescent"), 0.25);
}

TEST_F(ConfigurationTest, GetDisplacementInverse) {
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 5, "decrescent"), 0.5);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 10, "decrescent"), 0);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 0, "decrescent"), 1);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 7.5, "decrescent"), 0.25);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 2.5, "decrescent"), 0.75);
}

TEST_F(ConfigurationTest, GetDisplacementMedium) {
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 5, "medium"), 0.0);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 10, "medium"), 1);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 0, "medium"), 1);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 7.5, "medium"), 0.50);
    ASSERT_EQ(s.getDisplacement(Range(0, 10), 2.5, "medium"), 0.50);
    ASSERT_EQ(s.getDisplacement(Range(36.5, 37.5), 37.0, "medium"), 0);
    ASSERT_EQ(s.getDisplacement(Range(36.5, 37.5), 36.5, "medium"), 1);
    ASSERT_EQ(s.getDisplacement(Range(36.5, 37.5), 37.5, "medium"), 1);
}

TEST_F(ConfigurationTest, GetDisplacementInvalidArgument) {
    ASSERT_THROW(s.getDisplacement(Range(0, 10), 5, "kjflakj"), std::invalid_argument);
}

TEST_F(ConfigurationTest, Low) {        
    ASSERT_EQ(s.evaluateNumber(37), 0);
    ASSERT_EQ(s.evaluateNumber(37.5), 0.2);
    ASSERT_EQ(s.evaluateNumber(36.5), 0.2);
    ASSERT_EQ(s.evaluateNumber(36.75), 0.1);
}

TEST_F(ConfigurationTest, Medium0) {
    ASSERT_EQ(s.evaluateNumber(33), 0.65);
    ASSERT_EQ(s.evaluateNumber(35), 0.21);
}

TEST_F(ConfigurationTest, Medium1) {   
    ASSERT_EQ(s.evaluateNumber(37.6), 0.21);
    ASSERT_EQ(s.evaluateNumber(39), 0.65);
}

TEST_F(ConfigurationTest, High0) {
    ASSERT_EQ(s.evaluateNumber(20), 1);
    ASSERT_EQ(s.evaluateNumber(32), 0.66);
}

TEST_F(ConfigurationTest, High1) {            
    ASSERT_EQ(s.evaluateNumber(39.1), 0.66);
    ASSERT_EQ(s.evaluateNumber(43), 1.0);
}