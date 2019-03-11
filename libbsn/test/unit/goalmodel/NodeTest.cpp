#include <gtest/gtest.h>

#include "goalmodel/Node.hpp"

using namespace bsn::goalmodel;

class NodeTest : public testing::Test { 
    protected:
        NodeTest() {}

        virtual void SetUp() {
        }
};

TEST_F(NodeTest, SimpleConstruct) {
    std::string id = "G3_T1.11";
    std::string description = "Read data";

    Node node(id, description);

    ASSERT_EQ(node.getID(), id);
    ASSERT_EQ(node.getDescription(), description);
}

TEST_F(NodeTest, AddChild) {

    Node parentNode("G3_T1", "Read");
    Node childNode("G3_T1.4", "Read ABP");

    parentNode.addChild(childNode);

    ASSERT_EQ(parentNode.getChildren().size(), 1);
    EXPECT_TRUE(parentNode.getChildren().at(0)==childNode);

}

TEST_F(NodeTest, RemoveChild) {

    Node parentNode("G3_T1", "Read");
    Node childNode("G3_T1.4", "Read ABP");
    parentNode.addChild(childNode);

    parentNode.removeChild("G3_T1.4");

    ASSERT_EQ(parentNode.getChildren().size(), 0);
}

TEST_F(NodeTest, RemoveChildNotFound) {

    Node parentNode("G3_T1", "Read");
    Node childNode("G3_T1.4", "Read ABP");
    parentNode.addChild(childNode);

    try {
        parentNode.removeChild("XXX");
        FAIL() << "Expected our of range exception";
    }
    catch(std::out_of_range const & err) {
        EXPECT_EQ(err.what(),std::string("Child Not Found"));
    }
}

TEST_F(NodeTest, GetChild) {

    Node parentNode("G3_T1", "Read");
    Node childNode("G3_T1.4", "Read ABP");
    parentNode.addChild(childNode);

    Node returnedNode = parentNode.getChild("G3_T1.4");

    ASSERT_EQ(returnedNode.getID(), "G3_T1.4");
}

TEST_F(NodeTest, GetChildNotFound) {

    Node parentNode("G3_T1", "Read");
    Node childNode("G3_T1.4", "Read ABP");
    parentNode.addChild(childNode);

    try {
        Node returnedNode = parentNode.getChild("XXX");
        FAIL() << "Expected out of range exception";
    }
    catch(std::out_of_range const & err) {
        EXPECT_EQ(err.what(),std::string("Child Not Found"));
    }
}