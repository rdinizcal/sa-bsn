#include <gtest/gtest.h>
#include <stdint.h>

#include "bsn/generator/Markov.hpp"
#include "bsn/range/Range.hpp"

using namespace std;
using namespace bsn::range;
using namespace bsn::generator;

class MarkovTest : public testing::Test {
    protected:
        array<float,25> transitions;
        array<Range, 5> ranges;
        Markov m4, m0;

        MarkovTest() : transitions(), ranges() {}

        virtual void SetUp() {
            transitions = {
                0,100,0,0,0,
                0,0,100,0,0,
                0,0,0,100,0,
                0,0,0,0,100,
                100,0,0,0,0};

            ranges = {Range(1, 3), Range(4, 6), Range(7, 9), Range(10, 11), Range(12, 13)};

            m4.transitions = transitions;
            m4.ranges = ranges;
            m4.currentState = 4;

            m0.transitions = transitions;
            m0.ranges = ranges;
            m0.currentState = 0;
        }
};

TEST_F(MarkovTest, NextStates) {
    ASSERT_EQ(4, m4.currentState);
    m4.next_state();

    ASSERT_EQ(0, m4.currentState);
    m4.next_state();

    ASSERT_EQ(1, m4.currentState);
    m4.next_state();

    ASSERT_EQ(2, m4.currentState);
    m4.next_state();

    ASSERT_EQ(3, m4.currentState);
    m4.next_state();
}

TEST_F(MarkovTest, StateCalculations) {
    ASSERT_EQ(true, m0.ranges[m0.currentState].in_range(m0.calculate_state()));
    ASSERT_EQ(0, m0.currentState);
    m0.next_state();

    ASSERT_EQ(true, m0.ranges[m0.currentState].in_range(m0.calculate_state()));
    ASSERT_EQ(1, m0.currentState);
    m0.next_state();

    ASSERT_EQ(true, m0.ranges[m0.currentState].in_range(m0.calculate_state()));
    ASSERT_EQ(2, m0.currentState);
    m0.next_state();

    ASSERT_EQ(true, m0.ranges[m0.currentState].in_range(m0.calculate_state()));
    ASSERT_EQ(3, m0.currentState);
    m0.next_state();
    
    ASSERT_EQ(true, m0.ranges[m0.currentState].in_range(m0.calculate_state()));
    ASSERT_EQ(4, m0.currentState);
}