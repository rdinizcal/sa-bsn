#include <gtest/gtest.h>
#include <vector>

#include "libbsn/processor/Processor.hpp"

using namespace std;
using namespace bsn::processor;

class ProcessorTest : public testing::Test {
    protected:
        ProcessorTest() {}
};

TEST_F(ProcessorTest, GetThermometerId) {
    ASSERT_EQ(0, get_sensor_id("thermometer"));
}

TEST_F(ProcessorTest, GetECGId) {
    ASSERT_EQ(1, get_sensor_id("ecg"));
}

TEST_F(ProcessorTest, GetOxiemeterId) {
    ASSERT_EQ(2, get_sensor_id("oximeter"));
}

TEST_F(ProcessorTest, GetBPMSId) {
    ASSERT_EQ(3, get_sensor_id("abps"));

}

TEST_F(ProcessorTest, GetBPMDId) {
    ASSERT_EQ(4, get_sensor_id("abpd"));
}

TEST_F(ProcessorTest, GetGlucosemeterId) {
    ASSERT_EQ(5, get_sensor_id("glucosemeter"));
}

TEST_F(ProcessorTest, GetUnknownId) {
    ASSERT_EQ(-1, get_sensor_id("unknown"));
    ASSERT_EQ(-1, get_sensor_id("aaah"));
    ASSERT_EQ(-1, get_sensor_id("thermomemeter"));
}

TEST_F(ProcessorTest, GetValue) {
    ASSERT_EQ(10.5, get_value("tipo-10.5"));
}

TEST_F(ProcessorTest, FuseEmptyDataVector) {
    vector<double> vet({});

    ASSERT_EQ(-1, data_fuse(vet));
}

TEST_F(ProcessorTest, FuseThreeValues) {
    vector<double> vet({1,1,1});

    vet.at(0) = 70.0;
    vet.at(1) = 60.0;
    vet.at(2) = 50.0;

    ASSERT_EQ(66,static_cast<int>(data_fuse(vet)));
}