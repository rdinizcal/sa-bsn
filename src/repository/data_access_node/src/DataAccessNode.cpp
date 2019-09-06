#include "data_access_node/DataAccessNode.hpp"

DataAccessNode::DataAccessNode(int  &argc, char **argv) {}
DataAccessNode::~DataAccessNode() {}

void DataAccessNode::setUp() {}

void DataAccessNode::tearDown() {}

void DataAccessNode::run(){
    ros::spin();
}

void DataAccessNode::receiveInfo(const messages::Info::ConstPtr& msg) {
    /**
     *  ComponentData componentData = parse(msg->content) 
     *  persist componentData.toString();
     */
}

 