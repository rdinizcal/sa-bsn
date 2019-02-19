#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP

#include "ros/ros.h"
#include "bsn/TaskInfo.h"
#include "bsn/ContextInfo.h"

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

		static void receiveTaskInfo(const bsn::TaskInfo::ConstPtr& /*msg*/);
		static void receiveContextInfo(const bsn::ContextInfo::ConstPtr& /*msg*/);

    	void run();

  	private:
};

#endif 