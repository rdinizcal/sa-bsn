#include <gtest/gtest.h>

#include "libbsn/goalmodel/Property.hpp"

using namespace bsn::goalmodel;

class PropertyTest : public testing::Test {
    protected:
        PropertyTest() {}

        virtual void SetUp() {
        }
};

TEST_F(PropertyTest, SimpleConstruct) {
    std::string id = "W_G3_T1_11";
    double value = 0.7;

    Property property(id, value);

    ASSERT_EQ(property.getID(), id);
    ASSERT_EQ(property.getValue(), value);
}
