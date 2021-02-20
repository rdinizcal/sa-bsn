#include <gtest/gtest.h>
#include <memory>

#include "libbsn/goalmodel/Context.hpp"
#include "libbsn/goalmodel/Property.hpp"
#include "libbsn/goalmodel/Task.hpp"
#include "libbsn/goalmodel/LeafTask.hpp"
#include "libbsn/goalmodel/Node.hpp"
#include "libbsn/goalmodel/Goal.hpp"

using namespace bsn::goalmodel;

class TaskTest : public testing::Test {
    protected:
        TaskTest() {}

        virtual void SetUp() {
        }
};

TEST_F(TaskTest, SimpleConstruct) {
    std::string id = "G3_T1.11";
    std::string description = "Read data";

    Task task(id, description);

    ASSERT_EQ(task.getID(), id);
    ASSERT_EQ(task.getDescription(), description);
}

TEST_F(TaskTest, AddChild) {

    Task parentTask("G3_T1.41", "Read ABP");
    LeafTask childTask(std::string("G3_T1.412"), std::string("Read systolic"), Property("W_G3_T1_412",1), Property("R_G3_T1_412",1), Property("F_G3_T1_412",1));

    parentTask.addChild(std::make_shared<LeafTask>(childTask));

    ASSERT_EQ(parentTask.getChildren().size(), 1);
    EXPECT_TRUE(*(parentTask.getChildren().at(0))==childTask);

}

TEST_F(TaskTest, AddGoalAsChild) {

    Task parentTask("G3_T1.41", "Read ABP");
    Goal childGoal("G3_T1", "Read Sensor Info");

    try {
        parentTask.addChild(std::make_shared<Goal>(childGoal));
        FAIL() << "Expected not to be able to add a goal as child of a task";
    }
    catch(std::exception const & err) {
        EXPECT_EQ(err.what(),std::string("Tasks cannot contain goals as children"));
    }

}

TEST_F(TaskTest, RemoveChild) {

    Task parentTask("G3_T1.41", "Read ABP");
    LeafTask childTask(std::string("G3_T1.412"), std::string("Read systolic"), Property("W_G3_T1_412",1), Property("R_G3_T1_412",1), Property("F_G3_T1_412",1));
    parentTask.addChild(std::make_shared<LeafTask>(childTask));

    parentTask.removeChild("G3_T1.412");

    ASSERT_EQ(parentTask.getChildren().size(), 0);
}

TEST_F(TaskTest, RemoveChildNotFound) {

    Task parentTask("G3_T1.41", "Read ABP");
    LeafTask childTask(std::string("G3_T1.412"), std::string("Read systolic"), Property("W_G3_T1_412",1), Property("R_G3_T1_412",1), Property("F_G3_T1_412",1));
    parentTask.addChild(std::make_shared<LeafTask>(childTask));

    try {
        parentTask.removeChild("XXX");
        FAIL() << "Expected our of range exception";
    }
    catch(std::out_of_range const & err) {
        EXPECT_EQ(err.what(),std::string("Child Not Found"));
    }
}

TEST_F(TaskTest, GetChild) {

    Task parentTask("G3_T1.41", "Read ABP");
    LeafTask childTask(std::string("G3_T1.412"), std::string("Read systolic"), Property("W_G3_T1_412",1), Property("R_G3_T1_412",1), Property("F_G3_T1_412",1));
    parentTask.addChild(std::make_shared<LeafTask>(childTask));

    std::shared_ptr<Node> returnedTask = parentTask.getChild("G3_T1.412");

    ASSERT_EQ(returnedTask->getID(), "G3_T1.412");
}

TEST_F(TaskTest, GetChildNotFound) {

    Task parentTask("G3_T1.41", "Read ABP");
    LeafTask childTask(std::string("G3_T1.412"), std::string("Read systolic"), Property("W_G3_T1_412",1), Property("R_G3_T1_412",1), Property("F_G3_T1_412",1));
    parentTask.addChild(std::make_shared<LeafTask>(childTask));

    try {
        std::shared_ptr<Node> returnedTask = parentTask.getChild("XXX");
        FAIL() << "Expected out of range exception";
    }
    catch(std::out_of_range const & err) {
        EXPECT_EQ(err.what(),std::string("Child Not Found"));
    }
}