#include <gtest/gtest.h>

#include "libbsn/resource/Battery.hpp"

using namespace std;
using namespace bsn::resource;

class ResourcesTest : public testing::Test {
    protected:
        Battery resource;

        ResourcesTest() : resource() {}

        virtual void SetUp () {
            resource.setId("Battery");
            resource.setCapacity(10.0);
            resource.setCurrentLevel(10.0);
            resource.setUnit(0.1);
        }
};

TEST_F(ResourcesTest, BasicConstruct) {
    Battery resource;

    ASSERT_EQ(resource.getCapacity(), 100);
    ASSERT_EQ(resource.getCurrentLevel(), 100);
    ASSERT_EQ(resource.getUnit(), 1);
}

TEST_F(ResourcesTest, Construct) {
    ASSERT_EQ(resource.getId(), "Battery");
    ASSERT_EQ(resource.getCapacity(), 10);
    ASSERT_EQ(resource.getCurrentLevel(), 10);
    ASSERT_EQ(resource.getUnit(), 0.1);
}

TEST_F(ResourcesTest, InvalidArgumentConstruct) {
    ASSERT_THROW(Battery resource("Battery", 10, 15, 0.1),
                 std::invalid_argument);
}

TEST_F(ResourcesTest, ResourceConsume) {
    resource.consume(5);

    ASSERT_EQ(resource.getCurrentLevel(), 9.5);
}

TEST_F(ResourcesTest, ResourceGeneration) {
    resource.consume(5);
    resource.generate(2);

    ASSERT_EQ(resource.getCurrentLevel(),9.7);
}