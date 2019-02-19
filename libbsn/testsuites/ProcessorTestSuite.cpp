#include <gtest/gtest.h>
#include <vector>

#include "bsn/processor/Processor.hpp"

using namespace std;
using namespace bsn::processor;

class ProcessorTest : public testing::Test {
    protected:
        vector<list<double>> vet;
        vector<list<double>> empty;

        ProcessorTest() : vet(3), empty(3) {}

        virtual void SetUp () {}

        virtual void TearDown () {
            vet[0].clear();
            vet[1].clear();
            vet[2].clear();

            empty[0].clear();
            empty[1].clear();
            empty[2].clear();
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

TEST_F(ProcessorTest, AvailableToProcess) {
    vet[0].push_back(1.0);

    ASSERT_EQ(true, available_to_process(vet));
    vet[1].push_back(1.0);
    vet[2].push_back(1.0);
    ASSERT_EQ(true, available_to_process(vet));
    ASSERT_EQ(false, available_to_process(empty));
}

TEST_F(ProcessorTest, DataFuse) {
    ASSERT_EQ(-1, data_fuse(vet));
    vet[0].push_back(0.75);
    vet[1].push_back(0.60);

    vet[0].push_back(0.10);
    vet[1].push_back(0.20);
    //data_fuse
    ASSERT_EQ(-1, data_fuse(empty));
    ASSERT_EQ(0.675, data_fuse(vet));
    ASSERT_EQ(0.1, vet[0].front());
    ASSERT_EQ(0.2, vet[1].front());
}