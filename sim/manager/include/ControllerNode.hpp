#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <string>

class ControllerNode {

    private:
      	ControllerNode(const ControllerNode &);
    	ControllerNode &operator=(const ControllerNode &);

    	virtual void setUp();
    	virtual void tearDown();

  public:
    	ControllerNode(int &argc, char **argv, std::string name);
    	virtual ~ControllerNode();

		void ControllerNode::receiveTaskInfo(const std_msgs::String::ConstPtr& /*msg*/);
		void ControllerNode::receiveContextInfo(const std_msgs::String::ConstPtr& /*msg*/);

    	void run();

  	private:
};

#endif 