#include <cxxtest/TestSuite.h>
#include "bsn/range/Range.hpp"
#include "bsn/generator/Markov.hpp"
#include "bsn/filters/MovingAverage.hpp"
#include "bsn/configuration/SensorConfiguration.hpp"

#include <stdint.h>
#include <iostream>

using namespace std;
using namespace bsn::range;
using namespace bsn::filters;
using namespace bsn::configuration;
using namespace bsn::generator;

class IntegrationTestSuite : public CxxTest::TestSuite{    
  public:
    Range l;
    Range m1;
    Range m2;
    Range h1;
    Range h2;    

    array<Range,2> a1;
    array<Range,2> a2;
    array<Range,3> percentages;

	SensorConfiguration s;
    array<float,25> transitions;
    array<Range, 5> ranges;
    
    void setUp(){
        transitions =
        {
            0,100,0,0,0,
            0,0,100,0,0,
            0,0,0,100,0,
            0,0,0,0,100,
            100,0,0,0,0
        };

        ranges = {Range(30.0, 33.0), Range(33.1, 36.4), Range(36.5, 37.5), Range(37.6, 39.0), Range(39.1, 42.0)};        

        h1.setLowerBound(30.0);
		h1.setUpperBound(33.0);

		m1.setLowerBound(33.1);
		m1.setUpperBound(36.4);

        l.setLowerBound(36.5);
		l.setUpperBound(37.5);

        m2.setLowerBound(37.6);
		m2.setUpperBound(39.0);

        h2.setLowerBound(39.1);
		h2.setUpperBound(42.0);

        percentages = {Range(0,0.2), Range(0.21,0.65), Range(0.66,1.0)};
        a1 = {m1, m2};
        a2 = {h1, h2};		   
    }

    // Integração total do sistema
    void test_integration_low(){
        cout << "\nTestando integração do sistema\n";
        cout << "\tTestando integração do sistema para um sinal low:\n";

        MovingAverage avg(2);
        Markov m(transitions, ranges, 2);   
        TS_ASSERT_EQUALS(m.currentState,2);

        avg.insert(m.calculate_state(), "thermometer");
        avg.insert(m.calculate_state(), "thermometer");
        double data = avg.getValue("thermometer ");
        
        
        TS_ASSERT_LESS_THAN_EQUALS(data, 37.5);
        TS_ASSERT_LESS_THAN_EQUALS(36.5, data);
        
        SensorConfiguration s(1,l,a1,a2,percentages);
        TS_ASSERT_LESS_THAN_EQUALS(s.evaluateNumber(data), 0.2);
        TS_ASSERT_LESS_THAN_EQUALS(0.0, s.evaluateNumber(data));
    }

    void test_integration_high(){
        cout << "\tTestando integração do sistema para um sinal high:\n";

        MovingAverage avg(4);
        Markov m(transitions, ranges, 4);   
        TS_ASSERT_EQUALS(m.currentState,4);
        avg.insert(m.calculate_state(), "thermometer");
        avg.insert(m.calculate_state(), "thermometer");
        double data = avg.getValue("thermometer ");
        
        TS_ASSERT_LESS_THAN_EQUALS(data, 42);
        TS_ASSERT_LESS_THAN_EQUALS(39.1, data);
        
        SensorConfiguration s(1,l,a1,a2,percentages);
        TS_ASSERT_LESS_THAN_EQUALS(s.evaluateNumber(data), 1.0);
        TS_ASSERT_LESS_THAN_EQUALS(0.66, s.evaluateNumber(data));
    }

    void test_integration_medium(){
        cout << "\tTestando integração do sistema para um sinal medio:\n";

        MovingAverage avg(1);
        Markov m(transitions, ranges, 1);   
        TS_ASSERT_EQUALS(m.currentState,1);
        avg.insert(m.calculate_state(), "thermometer");
        avg.insert(m.calculate_state(), "thermometer");
        double data = avg.getValue("thermometer ");
        
        TS_ASSERT_LESS_THAN_EQUALS(data, 36.4);
        TS_ASSERT_LESS_THAN_EQUALS(33.1, data);
        
        SensorConfiguration s(1,l,a1,a2,percentages);        
        TS_ASSERT_LESS_THAN_EQUALS(s.evaluateNumber(data), 0.65);
        TS_ASSERT_LESS_THAN_EQUALS(0.21, s.evaluateNumber(data));
    }

};