#ifndef DATA_ACCESS_NODE_HPP
#define DATA_ACCESS_NODE_HPP

#include <string>
#include <fstream>
#include <vector>
#include <chrono>
#include <map>

#include "ComponentData.hpp"
#include "messages/Info.h"

#include "ros/ros.h"
#include <ros/package.h>

class DataAccessNode {

	public:
    	DataAccessNode(int &argc, char **argv);
    	virtual ~DataAccessNode();

    private:
      	DataAccessNode(const DataAccessNode &);
    	DataAccessNode &operator=(const DataAccessNode &);

	public:
		void setUp();
		void tearDown();
		void run();

		void parse(std::string, ComponentData&);
	  	void receiveInfo(const messages::Info::ConstPtr& /*msg*/);
		void persistComponentData();

		int64_t now() const;

  	private:
	    std::fstream fp;
		ros::NodeHandle handle;
		std::map<std::string, std::vector<ComponentData>> componentMap;
		std::string filepath;
};

#endif 