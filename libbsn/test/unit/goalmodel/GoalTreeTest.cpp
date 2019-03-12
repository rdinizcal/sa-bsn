#include <gtest/gtest.h>

#include "goalmodel/GoalTree.hpp"
#include "goalmodel/Goal.hpp"


using namespace bsn::goalmodel;

class GoalTreeTest : public testing::Test { 
    protected:
        GoalTreeTest() {}

        virtual void SetUp() {
        }
};

TEST_F(GoalTreeTest, SimpleConstruct) {
    std::string actor = "Body Sensor Network";

    GoalTree goaltree(actor);

    ASSERT_EQ(goaltree.getActor(), actor);
}

TEST_F(GoalTreeTest, AddRootGoal) {
    std::string actor = "Body Sensor Network";
    GoalTree goaltree(actor);
    Goal rgoal("G1", "Emergency is detected");

    goaltree.addRootGoal(rgoal);

    ASSERT_TRUE(goaltree.getNode("G1")==&rgoal);
    ASSERT_EQ(goaltree.getSize(),1);

}

TEST_F(GoalTreeTest, NotAllowMoreThanOneRootGoal) {
    std::string actor = "Body Sensor Network";
    GoalTree goaltree(actor);
    Goal rgoal1("G1", "Emergency is detected");
    Goal rgoal2("G2", "Patient is healthy");

    goaltree.addRootGoal(rgoal1);

    try {
        goaltree.addRootGoal(rgoal2);
        FAIL() << "Expected not to allow more than one root goal";
    }
    catch(std::invalid_argument const & err) {
        EXPECT_EQ(err.what(),std::string("No more than 1 root goals allowed"));
    }
}


TEST_F(GoalTreeTest, GetSpecificNode) {
    std::string actor = "Body Sensor Network";
    GoalTree goaltree(actor);
    Goal rgoal("G1", "Emergency is detected");
    goaltree.addRootGoal(rgoal);

    Node* new_goal = goaltree.getNode("G1");

    ASSERT_TRUE(new_goal==&rgoal);
}

TEST_F(GoalTreeTest, GetTreeSize) {
    std::string actor = "Body Sensor Network";
    GoalTree goaltree(actor);
    Goal rgoal("G1", "Emergency is detected");
    goaltree.addRootGoal(rgoal);

    int size = goaltree.getSize();

    ASSERT_TRUE(size == 1);
}

TEST_F(GoalTreeTest, AddGoalWParent) {
    std::string actor = "Body Sensor Network";
    GoalTree goaltree(actor);
    Goal goal1("G1", "Emergency is detected");
    goaltree.addRootGoal(goal1);

    Goal goal2("G2", "Patient status is monitored");
    goaltree.addNode(goal2, "G1");

    ASSERT_EQ(goaltree.getSize(),2);
    ASSERT_TRUE(goaltree.getNode("G2")==&goal2);
    ASSERT_TRUE((*goaltree.getNode("G1")).getChildren().size()==1);
    ASSERT_TRUE((*goaltree.getNode("G1")).getChild("G2")==goal2);
}

TEST_F(GoalTreeTest, AddGoalWNonExistentParent) {
    std::string actor = "Body Sensor Network";
    GoalTree goaltree(actor);
    Goal goal1("G1", "Emergency is detected");
    goaltree.addRootGoal(goal1);
    Goal goal2("G2", "Patient status is monitored");

    try {
        goaltree.addNode(goal2, "GX");
        FAIL() << "Expected not to allow an insertion of goal without parent";
    }
    catch(std::out_of_range const & err) {
        EXPECT_EQ(err.what(),std::string("Could not find node"));
    }

}

TEST_F(GoalTreeTest, AddGoalWChildren) {
    std::string actor = "Body Sensor Network";
    GoalTree goaltree(actor);
    Goal goal1("G1", "Emergency is detected");
    goaltree.addRootGoal(goal1);

    Goal goal2("G2", "Patient status is monitored");
    Goal goal3("G3", "Vital signs are monitored");
    Goal goal4("G4", "Vital signs are analyzed");

    goal2.addChild(goal3);
    goal2.addChild(goal4);
    goaltree.addNode(goal2, "G1");

    ASSERT_EQ(goaltree.getSize(),4);
    ASSERT_TRUE(goaltree.getNode("G2")==&goal2);
    ASSERT_EQ((*goaltree.getNode("G2")).getChildren().size(),2);
    ASSERT_TRUE((*goaltree.getNode("G2")).getChild("G3")==goal3);
    ASSERT_TRUE((*goaltree.getNode("G2")).getChild("G4")==goal4);
}