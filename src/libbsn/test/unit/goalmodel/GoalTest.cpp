#include <gtest/gtest.h>
#include <memory>

#include "libbsn/goalmodel/Context.hpp"
#include "libbsn/goalmodel/Property.hpp"
#include "libbsn/goalmodel/Goal.hpp"
#include "libbsn/goalmodel/Node.hpp"

using namespace bsn::goalmodel;

class GoalTest : public testing::Test { 
    protected:
        GoalTest() {}

        virtual void SetUp() {
        }
};

TEST_F(GoalTest, SimpleConstruct) {
    std::string id = "G3_T1.11";
    std::string description = "Read data";

    Goal goal(id, description);

    ASSERT_EQ(goal.getID(), id);
    ASSERT_EQ(goal.getDescription(), description);
}

TEST_F(GoalTest, AddChild) {

    Goal parentGoal("G3_T1", "Read");
    Goal childGoal("G3_T1.4", "Read ABP");
    
    //childGoal->setID("G3_T1.4");
    //childGoal->setDescription("Read ABP");

    parentGoal.addChild(std::make_shared<Goal>(childGoal));

    ASSERT_EQ(parentGoal.getChildren().size(), 1);
    EXPECT_TRUE(*(parentGoal.getChildren().at(0))==childGoal);

}

TEST_F(GoalTest, RemoveChild) {

    Goal parentGoal("G3_T1", "Read");
    Goal childGoal("G3_T1.4", "Read ABP");
    parentGoal.addChild(std::make_shared<Goal>(childGoal));

    parentGoal.removeChild("G3_T1.4");

    ASSERT_EQ(parentGoal.getChildren().size(), 0);
}

TEST_F(GoalTest, RemoveChildNotFound) {

    Goal parentGoal("G3_T1", "Read");
    Goal childGoal("G3_T1.4", "Read ABP");
    parentGoal.addChild(std::make_shared<Goal>(childGoal));

    try {
        parentGoal.removeChild("XXX");
        FAIL() << "Expected out of range exception";
    }
    catch(std::out_of_range const & err) {
        EXPECT_EQ(err.what(),std::string("Child Not Found"));
    }
}

TEST_F(GoalTest, GetChild) {

    Goal parentGoal("G3_T1", "Read");
    Goal childGoal("G3_T1.4", "Read ABP");
    parentGoal.addChild(std::make_shared<Goal>(childGoal));

    std::shared_ptr<Node> returnedGoal = parentGoal.getChild("G3_T1.4");

    ASSERT_EQ(returnedGoal->getID(), "G3_T1.4");
}

TEST_F(GoalTest, GetChildNotFound) {

    Goal parentGoal("G3_T1", "Read");
    Goal childGoal("G3_T1.4", "Read ABP");
    parentGoal.addChild(std::make_shared<Goal>(childGoal));

    try {
        std::shared_ptr<Node> returnedGoal = parentGoal.getChild("XXX");
        FAIL() << "Expected out of range exception";
    }
    catch(std::out_of_range const & err) {
        EXPECT_EQ(err.what(),std::string("Child Not Found"));
    }
}