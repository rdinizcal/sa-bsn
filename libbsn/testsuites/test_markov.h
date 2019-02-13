#include <cxxtest/TestSuite.h>
#include <iostream>
#include <array>

#include "bsn/generator/Markov.hpp"
#include "bsn/range/Range.hpp"


using namespace std;
using namespace bsn::range;
using namespace bsn::generator;

class RangesTestSuite : public CxxTest::TestSuite {
    array<float,25> transitions;
    array<Range, 5> ranges;

    public:

    void setUp() {
        transitions =
        {
            0,100,0,0,0,
            0,0,100,0,0,
            0,0,0,100,0,
            0,0,0,0,100,
            100,0,0,0,0
        };

        ranges = {Range(1, 3), Range(4, 6), Range(7, 9), Range(10, 11), Range(12, 13)};
    }
	void test_next_states(){
        cout << "\nTestando Markov\n";
		cout << "\tTestando passagem de estados:\n";

        Markov m(transitions, ranges, 4);
        
        TS_ASSERT_EQUALS(4, m.currentState);
        m.next_state();
        TS_ASSERT_EQUALS(0, m.currentState);
        m.next_state();
        TS_ASSERT_EQUALS(1, m.currentState);
        m.next_state();
        TS_ASSERT_EQUALS(2, m.currentState);
        m.next_state();
        TS_ASSERT_EQUALS(3, m.currentState);    
        m.next_state();		
	}
	void test_state_calculations(){
		cout << "\tTestando intervalos dos estados:\n";

        Markov m(transitions, ranges, 0);
        // Testando se o valor realmente corresponde ao intervalo do estado      
		TS_ASSERT_EQUALS(true, m.ranges[m.currentState].in_range(m.calculate_state()));
        TS_ASSERT_EQUALS(0, m.currentState);
        m.next_state();        
        TS_ASSERT_EQUALS(true, m.ranges[m.currentState].in_range(m.calculate_state()));
        TS_ASSERT_EQUALS(1, m.currentState);
        m.next_state();
        TS_ASSERT_EQUALS(true, m.ranges[m.currentState].in_range(m.calculate_state()));
        TS_ASSERT_EQUALS(2, m.currentState);
        m.next_state();
        TS_ASSERT_EQUALS(true, m.ranges[m.currentState].in_range(m.calculate_state()));
        TS_ASSERT_EQUALS(3, m.currentState);
        m.next_state();
        TS_ASSERT_EQUALS(true, m.ranges[m.currentState].in_range(m.calculate_state()));
        TS_ASSERT_EQUALS(4, m.currentState);
	}
};