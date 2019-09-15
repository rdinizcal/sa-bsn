#ifndef DATA_ACCESS_NODE_HPP
#define DATA_ACCESS_NODE_HPP

#include <string>
#include <fstream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <map>

#include "ComponentData.hpp"
#include "messages/Info.h"
#include "messages/LearningData.h"
#include "services/ControllerInfo.h"

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

		bool sendInfo(services::ControllerInfo::Request &req, services::ControllerInfo::Response &res);

		void parse(std::string, ComponentData&);
		void finishCycle(double &timestamp);
	  	void receiveInfo(const messages::Info::ConstPtr& /*msg*/);
		void sendLearningInfo();
		void persistComponentData();

		int64_t now() const;

  	private:
	    std::fstream fp;
		ros::NodeHandle handle;
		ros::ServiceServer info_service;
		std::map<std::string, std::vector<ComponentData>> componentMap;
		std::vector<std::string> components_in_cycle;
		ros::Publisher info_dataaccessnode2learning_pub;
		std::string filepath;
		int logical_clock;
};

#endif 