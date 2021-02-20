#include <gtest/gtest.h>
#include <memory>

#include "libbsn/goalmodel/Node.hpp"

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

    parentNode.addChild(std::make_shared<Node>(childNode));

    ASSERT_EQ(parentNode.getChildren().size(), 1);
    EXPECT_TRUE(*(parentNode.getChild("G3_T1.4")) == childNode);

}

TEST_F(NodeTest, RemoveChild) {

    Node parentNode("G3_T1", "Read");
    Node childNode("G3_T1.4", "Read ABP");
    parentNode.addChild(std::make_shared<Node>(childNode));

    parentNode.removeChild("G3_T1.4");

    ASSERT_EQ(parentNode.getChildren().size(), 0);
}

TEST_F(NodeTest, RemoveChildNotFound) {

    Node parentNode("G3_T1", "Read");
    Node childNode("G3_T1.4", "Read ABP");
    parentNode.addChild(std::make_shared<Node>(childNode));

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
    parentNode.addChild(std::make_shared<Node>(childNode));

    std::shared_ptr<Node> returnedNode = parentNode.getChild("G3_T1.4");

    ASSERT_EQ(returnedNode->getID(), "G3_T1.4");
}

TEST_F(NodeTest, GetChildNotFound) {

    Node parentNode("G3_T1", "Read");
    Node childNode("G3_T1.4", "Read ABP");
    parentNode.addChild(std::make_shared<Node>(childNode));

    try {
        std::shared_ptr<Node> returnedNode = parentNode.getChild("XXX");
        FAIL() << "Expected out of range exception";
    }
    catch(std::out_of_range const & err) {
        EXPECT_EQ(err.what(),std::string("Child Not Found"));
    }
}

TEST_F(NodeTest, AddChildWChild) {

    Node parentNode("G3_T1", "Read");
    Node childNode("G3_T1.4", "Read ABP");
    Node grandchildNode("G3_T1.41", "Read Dyastolic");

    childNode.addChild(std::make_shared<Node>(grandchildNode));
    parentNode.addChild(std::make_shared<Node>(childNode));

    ASSERT_EQ(parentNode.getChildren().size(), 1);
    EXPECT_TRUE(*(parentNode.getChild("G3_T1.4"))==childNode);
    ASSERT_EQ(childNode.getChildren().size(), 1);
    EXPECT_TRUE(*(childNode.getChild("G3_T1.41"))==grandchildNode);
    EXPECT_TRUE(*(parentNode.getChild("G3_T1.4")->getChild("G3_T1.41"))==grandchildNode);

}

TEST_F(NodeTest, AddChildWChildrenWChild) {

    Node parentNode("Parent", "Read");
    Node boyChildNode("Boy Child", "Read ABPb");
    Node girlChildNode("Girl Child", "Read ABPg");
    Node grandchildNode("Boy GrandChild", "Read Dyastolic 1");
    Node greatgrandchildNode("Boy GreatGrandChild", "Read Dyastolic 1.7");

    grandchildNode.addChild(std::make_shared<Node>(greatgrandchildNode));
    boyChildNode.addChild(std::make_shared<Node>(grandchildNode));
    parentNode.addChild(std::make_shared<Node>(girlChildNode));    
    parentNode.addChild(std::make_shared<Node>(boyChildNode));

    ASSERT_EQ(parentNode.getChildren().size(), 2);
    EXPECT_TRUE(*(parentNode.getChild("Boy Child"))==boyChildNode);
    EXPECT_TRUE(*(parentNode.getChild("Girl Child"))==girlChildNode);
    ASSERT_EQ(boyChildNode.getChildren().size(), 1);
    ASSERT_EQ(girlChildNode.getChildren().size(), 0);
    EXPECT_TRUE(*(boyChildNode.getChild("Boy GrandChild"))==grandchildNode);


    EXPECT_TRUE(*(parentNode.getChild("Boy Child")->getChild("Boy GrandChild"))==grandchildNode);
    EXPECT_TRUE(*(parentNode.getChild("Boy Child")->getChild("Boy GrandChild")->getChild("Boy GreatGrandChild"))==greatgrandchildNode);

}