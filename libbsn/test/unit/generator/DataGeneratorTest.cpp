#include <gtest/gtest.h>
#include <stdint.h>

#include "generator/DataGenerator.hpp"
#include "generator/Markov.hpp"
#include "range/Range.hpp"

using namespace bsn::range;
using namespace bsn::generator;

class DataGeneratorTest : public testing::Test {
    protected:
        std::array<float,25> transitions;
        std::array<Range, 5> states;

        DataGeneratorTest() : transitions(), states() {}

        virtual void SetUp() {
            transitions = {{
                0,100,0,0,0,
                0,0,100,0,0,
                0,0,0,100,0,
                0,0,0,0,100,
                100,0,0,0,0}};

            states = {{Range(1, 3), Range(4, 6), Range(7, 9), Range(10, 11), Range(12, 13)}};
        }
};

TEST_F(DataGeneratorTest, GetValue) {
    Markov m4;
    m4.transitions = transitions;
    m4.states = states;
    m4.currentState = 4;

    DataGenerator dg(m4);
    
    double x = dg.getValue();
    ASSERT_TRUE(1 <= x && x <= 3);
}