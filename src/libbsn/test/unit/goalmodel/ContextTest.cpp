#include <gtest/gtest.h>

#include "libbsn/goalmodel/Context.hpp"

using namespace bsn::goalmodel;

class ContextTest : public testing::Test {
    protected:
        ContextTest() {}

        virtual void SetUp() {
        }
};

TEST_F(ContextTest, SimpleConstruct) {
    std::string id = "CTX_G3_T1_1";
    std::string description = "SaO2_available";
    double value = false;

    Context context(id, description, value);

    ASSERT_EQ(context.getID(), id);
    ASSERT_EQ(context.getDescription(), description);
    ASSERT_EQ(context.getValue(), value);
}
