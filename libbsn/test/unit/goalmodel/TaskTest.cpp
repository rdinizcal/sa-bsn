#include <gtest/gtest.h>

#include "goalmodel/Context.hpp"
#include "goalmodel/Property.hpp"
#include "goalmodel/Task.hpp"

using namespace bsn::goalmodel;

class TaskTest : public testing::Test {
    protected:
        TaskTest() {}

        virtual void SetUp() {
        }
};

TEST_F(TaskTest, SimpleConstructNoChildren) {
    std::string id = "G3_T1.11";
    std::string description = "Read data";
    Context context("CTX_G3_T1_1","SaO2_available",false);
    Property cost("W_G3_T1_11",1);
    Property reliability("R_G3_T1_11",1);
    Property frequency("F_G3_T1_11",1);

    Task task(id, description, context, cost, reliability, frequency);

    ASSERT_EQ(task.getID(), id);
    ASSERT_EQ(task.getDescription(), description);
    EXPECT_TRUE(task.getContext()==context);
    EXPECT_TRUE(task.getCost()==cost);
    EXPECT_TRUE(task.getReliability()==reliability);
    EXPECT_TRUE(task.getFrequency()==frequency);

}
