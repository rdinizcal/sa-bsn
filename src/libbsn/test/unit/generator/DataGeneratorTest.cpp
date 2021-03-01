#include <gtest/gtest.h>
#include <stdint.h>
#include <stdexcept>

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

TEST_F(DataGeneratorTest, GetValueWithOutOfBoundsState) {
    mk.currentState = 10;
    DataGenerator dg(mk);
    
    try{
        double x = dg.getValue();
        FAIL();
    } catch (const std::exception& e) {
        ASSERT_TRUE(true);
    }
}

TEST_F(DataGeneratorTest, NextState) {
    DataGenerator dg(mk);
    
    dg.nextState();
    /* Ideally:
     * ASSERT_TRUE(dg.markovChain.getCurrentState()==0);
     * but dg.markovChain is not accessible
     */
    double x = dg.getValue();

    //should generate a value from state 0 (the next state according to transition matrix)
    ASSERT_TRUE(states[0].getLowerBound() <= x && x <= states[0].getUpperBound());
}

TEST_F(DataGeneratorTest, NextButSameState) {
    transitions = {{
                0,100,0,0,0,
                0,0,100,0,0,
                0,0,0,100,0,
                0,0,0,0,100,
                0,0,0,0,100}};
    mk.transitions = transitions;
    DataGenerator dg(mk);
    
    dg.nextState();
    double x = dg.getValue();

    //should generate a value from state 0 (the next state according to transition matrix)
    ASSERT_TRUE(states[4].getLowerBound() <= x && x <= states[4].getUpperBound());
}