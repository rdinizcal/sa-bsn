#include <gtest/gtest.h>
#include <vector>

#include "processor/Processor.hpp"

using namespace std;
using namespace bsn::processor;

class ProcessorTest : public testing::Test {
    protected:
        vector<double> vet;
        vector<double> empty;

        ProcessorTest() : vet({-1,-1,-1}), empty(3) {}

        virtual void SetUp () {}

        virtual void TearDown () {
            vet.clear();
            empty.clear();
        }
};

TEST_F(ProcessorTest, GetSensorId) {
    ASSERT_EQ(0, get_sensor_id("thermometer"));
    ASSERT_EQ(1, get_sensor_id("ecg"));
    ASSERT_EQ(2, get_sensor_id("oximeter"));
    ASSERT_EQ(3, get_sensor_id("bpms"));
    ASSERT_EQ(4, get_sensor_id("bpmd"));
    ASSERT_EQ(-1, get_sensor_id("unknown"));
}

TEST_F(ProcessorTest, GetValue) {
    ASSERT_EQ(10.5, get_value("tipo-10.5"));
}

TEST_F(ProcessorTest, DataFuse) {
    ASSERT_EQ(-1, data_fuse(vet));
    vet.at(0) = 70.0;
    vet.at(1) = 60.0;
    vet.at(2) = 50.0;

    //data_fuse
    ASSERT_EQ(-1, data_fuse(empty));
    ASSERT_EQ(66,static_cast<int>(data_fuse(vet)));
}