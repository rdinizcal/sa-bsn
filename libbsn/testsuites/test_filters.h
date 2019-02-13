#include <cxxtest/TestSuite.h>

#include <stdint.h>
#include <iostream>

#include "bsn/filters/MovingAverage.hpp"

using namespace std;
using namespace bsn::filters;

class FiltersTestSuite: public CxxTest::TestSuite{    
  	public:
	
  	void test_initial_avg() {
    	cout << "Testando os Filtros\n";
		MovingAverage avg(1);
		cout << "\tTestando uma condição inicial 0\n";
		TS_ASSERT_EQUALS(avg.getValue("thermometer"), 0.0);
	}

  	void test_1_avg() {
    	cout << "\tTestando com 1 valor\n";
		MovingAverage avg(1);
		avg.insert(1, "thermometer");
		TS_ASSERT_EQUALS(avg.getValue("thermometer"), 1.0);
	}

  	void test_avg() {
    	cout << "\tTestando de forma geral com 5 valores\n";
    	MovingAverage avg(5);
		avg.insert(32.0, "thermometer");
		avg.insert(35.0, "thermometer");
		avg.insert(36.0, "thermometer");
		avg.insert(34.0, "thermometer");
		avg.insert(35.0, "thermometer");
		TS_ASSERT_EQUALS(avg.getValue("thermometer"), 34.4);
  	}

	void test_avg_with_less_values() {
		cout << "\tTestando com um numero menor de valores fornecidos\n";
		MovingAverage avg(5);
		avg.insert(32.0, "thermometer");
		avg.insert(35.0, "thermometer");
		avg.insert(36.0, "thermometer");
		avg.insert(34.0, "thermometer");
		TS_ASSERT_EQUALS(avg.getValue("thermometer"), 34.0);
	}

	void test_avg_with_more_values() {
		cout << "\tTestando com um numero maior de valores fornecidos\n";
		MovingAverage avg(5);
		avg.insert(200, "thermometer");
		avg.insert(32.0, "thermometer");
		avg.insert(35.0, "thermometer");
		avg.insert(36.0, "thermometer");
		avg.insert(34.0, "thermometer");
		avg.insert(35.0, "thermometer");
		avg.insert(37.0, "thermometer");
		TS_ASSERT_EQUALS(avg.getValue("thermometer"), 35.4);
	}

	void test_avg_with_5_types_of_sensors() {
		cout << "\tTestando com os 5 tipos de sensores diferentes\n";
		MovingAverage avg(5);
		avg.insert(36.5, "thermometer");
		avg.insert(80.0, "ecg");
		avg.insert(100.0, "oximeter");
		avg.insert(12.0, "bpms");
		avg.insert(8.0, "bpmd");
		TS_ASSERT_EQUALS(avg.getValue("thermometer"), 36.5);
		TS_ASSERT_EQUALS(avg.getValue("ecg"), 80.0);
		TS_ASSERT_EQUALS(avg.getValue("oximeter"), 100);
		TS_ASSERT_EQUALS(avg.getValue("bpms"), 12.0);
		TS_ASSERT_EQUALS(avg.getValue("bpmd"), 8.0);
	}

	void test_avg_with_5_types_of_sensors_with_two_values() {
		cout << "\tTestando com os 5 tipos de sensores diferentes com dois valores cada\n";
		MovingAverage avg(2);
		avg.insert(36.5, "thermometer");
		avg.insert(37.5, "thermometer");
		avg.insert(80.0, "ecg");
		avg.insert(100.0, "ecg");
		avg.insert(90.0, "oximeter");
		avg.insert(100.0, "oximeter");
		avg.insert(12.0, "bpms");
		avg.insert(14.0, "bpms");
		avg.insert(8.0, "bpmd");
		avg.insert(10.0, "bpmd");
		TS_ASSERT_EQUALS(avg.getValue("thermometer"), 37.0);
		TS_ASSERT_EQUALS(avg.getValue("ecg"), 90.0);
		TS_ASSERT_EQUALS(avg.getValue("oximeter"), 95);
		TS_ASSERT_EQUALS(avg.getValue("bpms"), 13.0);
		TS_ASSERT_EQUALS(avg.getValue("bpmd"), 9.0);
	}

};