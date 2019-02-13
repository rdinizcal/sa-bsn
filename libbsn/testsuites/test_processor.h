#include <cxxtest/TestSuite.h>
#include <iostream>
#include <array>

#include "bsn/processor/Processor.hpp"


using namespace std;
using namespace bsn::processor;

class RangesTestSuite : public CxxTest::TestSuite {
  public:  	
	void test_get_sensor_id(){
	    cout << "\tTestando metodo get_sensor_id:\n";
        TS_ASSERT_EQUALS(0,  get_sensor_id("thermometer"));
        TS_ASSERT_EQUALS(1,  get_sensor_id("ecg"));
        TS_ASSERT_EQUALS(2,  get_sensor_id("oximeter"));
        TS_ASSERT_EQUALS(3,  get_sensor_id("bpms"));
        TS_ASSERT_EQUALS(4,  get_sensor_id("bpmd"));
        TS_ASSERT_EQUALS(-1, get_sensor_id("unknown"));
	}
    void test_get_value(){
	    cout << "\tTestando metodo get_value:\n";
        string a = "tipo-10.5";
        TS_ASSERT_EQUALS(10.5, get_value(a));
	}
    void test_available_to_process() {
        cout << "\tTestando metodo available_to_process:\n";
        vector<list<double>> vet(3);
        vector<list<double>> empty(3);
        vet[0].push_back(1.0);
        
        TS_ASSERT_EQUALS(true, available_to_process(vet));
        vet[1].push_back(1.0);
        vet[2].push_back(1.0);
        TS_ASSERT_EQUALS(true, available_to_process(vet));
        TS_ASSERT_EQUALS(false, available_to_process(empty));
    }
    void test_data_fuse() {
        cout << "\tTestando metodo data_fuse:\n";
        vector<list<double>> vet(3);
        vector<list<double>> empty(3);
        TS_ASSERT_EQUALS(-1, data_fuse(vet));
        vet[0].push_back(0.75);
        vet[1].push_back(0.60);

        vet[0].push_back(0.10);
        vet[1].push_back(0.20);        
        //data_fuse
        TS_ASSERT_EQUALS(-1, data_fuse(empty));
        TS_ASSERT_EQUALS(0.675, data_fuse(vet));        
        TS_ASSERT_EQUALS(0.1, vet[0].front());
        TS_ASSERT_EQUALS(0.2, vet[1].front());
    }
	
};