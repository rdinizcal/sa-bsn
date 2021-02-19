#include <gtest/gtest.h>
#include <memory>

#include "libbsn/goalmodel/Context.hpp"
#include "libbsn/goalmodel/Property.hpp"
#include "libbsn/goalmodel/LeafTask.hpp"

using namespace bsn::goalmodel;

class LeafTaskTest : public testing::Test {
    protected:
        LeafTaskTest() {}

        virtual void SetUp() {
        }
};

TEST_F(LeafTaskTest, SimpleConstruct) {
    std::string id = "G3_T1.11";
    std::string description = "Read data";
    Context context("CTX_G3_T1_1","SaO2_available",false);
    Property cost("W_G3_T1_11",1);
    Property reliability("R_G3_T1_11",1);
    Property frequency("F_G3_T1_11",1);

    LeafTask leafTask(id, description, context, cost, reliability, frequency);

    ASSERT_EQ(leafTask.getID(), id);
    ASSERT_EQ(leafTask.getDescription(), description);
    EXPECT_TRUE(leafTask.getContext()==context);
    EXPECT_TRUE(leafTask.getCost()==cost);
    EXPECT_TRUE(leafTask.getReliability()==reliability);
    EXPECT_TRUE(leafTask.getFrequency()==frequency);
}

TEST_F(LeafTaskTest, SimpleConstructNoContext) {
    std::string id = "G3_T1.11";
    std::string description = "Read data";
    Property cost("W_G3_T1_11",1);
    Property reliability("R_G3_T1_11",1);
    Property frequency("F_G3_T1_11",1); 

    LeafTask leafTask(id, description, cost, reliability, frequency);

    ASSERT_EQ(leafTask.getID(), id);
    ASSERT_EQ(leafTask.getDescription(), description);
    EXPECT_TRUE(leafTask.getCost()==cost);
    EXPECT_TRUE(leafTask.getReliability()==reliability);
    EXPECT_TRUE(leafTask.getFrequency()==frequency);
}

TEST_F(LeafTaskTest, PreventAddChild) {

    LeafTask parentTask(std::string("G3_T1.412"), std::string("Read systolic"), Property("W_G3_T1_412",1), Property("R_G3_T1_412",1), Property("F_G3_T1_412",1));
    LeafTask childTask(std::string("G3_T1.412"), std::string("Read systolic"), Property("W_G3_T1_412",1), Property("R_G3_T1_412",1), Property("F_G3_T1_412",1));

    try {
        parentTask.addChild(std::make_shared<LeafTask>(childTask));
        FAIL() << "Expected not to be able to add a child to a leaf task";
    }
    catch(std::exception const & err) {
        EXPECT_EQ(err.what(),std::string("Leaf Tasks cannot contain children"));
    }

}