#ifndef DATA_ACCESS_NODE_HPP
#define DATA_ACCESS_NODE_HPP

#include <string>
#include "ComponentData.hpp"
#include "messages/Info.h"

#include "ros/ros.h"

class DataAccessNode {

	public:
    	DataAccessNode(int &argc, char **argv);
    	virtual ~DataAccessNode();

    private:
      	DataAccessNode(const DataAccessNode &);
    	DataAccessNode &operator=(const DataAccessNode &);

	public:
		void setUp();
		void run();

	  	void receiveInfo(const messages::Info::ConstPtr& /*msg*/);

  	private:
		ros::NodeHandle handle;
		std::map<std::string, std::vector<ComponentData>> componentMap;
};

#endif 