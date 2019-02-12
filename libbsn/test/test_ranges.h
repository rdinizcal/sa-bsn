#include <cxxtest/TestSuite.h>
#include <iostream>

#include "bsn/range/Range.hpp"

using namespace std;
using namespace bsn::range;

class RangesTestSuite: public CxxTest::TestSuite{    
  	public:
  	void test_constructor() {
		  cout << "Testando intervalos" << endl;
		  cout << "\tTestando construtor:\n";
		  Range r(2.5,5.0);
		  TS_ASSERT_EQUALS(r.getLowerBound(),2.5);
		  TS_ASSERT_EQUALS(r.getUpperBound(),5.0);
    }
	void test_illegal_construction(){
		cout << "\tTestando construtor com argumentos ilegais:\n";
		TS_ASSERT_THROWS(Range r(7.5,5.0), std::invalid_argument);
	}
	void test_in_Range(){
		cout << "\tTestando dentro do intervalo:\n";
		Range r(2.5,5.0);
		TS_ASSERT_EQUALS(true, r.in_range(3.0));
	}
	void test_out_Range(){
		cout << "\tTestando fora do intervalo:\n";
		Range r(2.5,5.0);
		TS_ASSERT_EQUALS(false, r.in_range(8.0));
	}
	void test_convert() {
		cout << "\tTestando conversão de escalas:\n";
		Range r(0.0,10.0);
		double result = r.convert(2,3,10.0);
		TS_ASSERT_EQUALS(3.0, result);
	}
};