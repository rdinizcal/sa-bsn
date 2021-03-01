#include <gtest/gtest.h>
#include <stdint.h>

#include "libbsn/generator/DataGenerator.hpp"
#include "libbsn/generator/Markov.hpp"
#include "libbsn/range/Range.hpp"

using namespace bsn::range;
using namespace bsn::generator;

class DataGeneratorTest : public testing::Test {
    protected:
        std::array<float,25> transitions;
        std::array<Range, 5> states;
        Markov mk;

        DataGeneratorTest() : transitions(), states(), mk() {}

        virtual void SetUp() {
            transitions = {{
                0,100,0,0,0,
                0,0,100,0,0,
                0,0,0,100,0,
                0,0,0,0,100,
                100,0,0,0,0}};

            states = {{Range(1, 3), Range(4, 6), Range(7, 9), Range(10, 11), Range(12, 13)}};
            mk.transitions = transitions;
            mk.states = states;
            mk.currentState = 4;
        }
};

TEST_F(DataGeneratorTest, GetValue) {
    DataGenerator dg(mk);
    
    for(int i=0; i < 10; i++){
        double x = dg.getValue();
        if ( !(states[4].getLowerBound() <= x && x <= states[4].getUpperBound()) )
            FAIL();
            return;
    }

    ASSERT_TRUE(true);
}

TEST_F(DataGeneratorTest, GetValueWithWrongCurrentState) {
    mk.currentState = 3;
    int wrong_state = 4;
    DataGenerator dg(mk);
    
    double x = dg.getValue();

    ASSERT_FALSE(states[wrong_state].getLowerBound() <= x && x <= states[wrong_state].getUpperBound());
}