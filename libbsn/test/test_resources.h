#include <cxxtest/TestSuite.h>
#include <stdexcept>
#include <string>

#include "bsn/resource/Resource.hpp"

using namespace std;
using namespace bsn::resource;

class ResourcesTestSuite : public CxxTest::TestSuite {
  
  public:  

    void test_basic_construct(){
        Resource resource;

        TS_ASSERT_EQUALS(true, resource.getCapacity() == 100);
        TS_ASSERT_EQUALS(true, resource.getCurrentLevel() == 100);
        TS_ASSERT_EQUALS(true, resource.getUnit() == 1);
    }

    void test_construct(){
        std::string name = "Battery";
        double cap = 10;
        double currLevel = 10;
        double unit = 0.1;

        Resource resource(name, cap, currLevel, unit);

        TS_ASSERT_EQUALS(true, resource.getName() == "Battery");
        TS_ASSERT_EQUALS(true, resource.getCapacity() == 10);
        TS_ASSERT_EQUALS(true, resource.getCurrentLevel() == 10);
        TS_ASSERT_EQUALS(true, resource.getUnit() == 0.1);
    }

    void test_invalid_arg_construct(){
        std::string name = "Battery";
        double cap = 10;
        double currLevel = 15;
        double unit = 0.1;

        TS_ASSERT_THROWS_EQUALS(Resource resource(name, cap, currLevel, unit),
                                                    std::invalid_argument i,true,true);
    }  

    void test_resource_cons(){
        std::string name = "Battery";
        double cap = 10;
        double currLevel = 10;
        double unit = 0.1;

        Resource resource(name, cap, currLevel, unit);

        resource.consume(5);

        TS_ASSERT_EQUALS(true, resource.getCurrentLevel() == 9.5);
    } 

    void test_resource_gen(){
        std::string name = "Battery";
        double cap = 10;
        double currLevel = 10;
        double unit = 0.1;

        Resource resource(name, cap, currLevel, unit);

        resource.consume(5);
        resource.generate(2);

        TS_ASSERT_EQUALS(true, resource.getCurrentLevel() == 9.7);
    } 

};