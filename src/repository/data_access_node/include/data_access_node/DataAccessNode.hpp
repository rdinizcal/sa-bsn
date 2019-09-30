#ifndef DATA_ACCESS_NODE_HPP
#define DATA_ACCESS_NODE_HPP

#include <string>
#include <fstream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <map>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include "ComponentData.hpp"
#include "messages/Info.h"
#include "messages/LearningData.h"
#include "messages/ControllerInfo.h"

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

		void sendControllerInfo();

		void parse(std::string, ComponentData&);
		void finishCycle(double &timestamp);
	  	void receiveInfo(const messages::Info::ConstPtr& /*msg*/);
		//void sendLearningInfo();
		void persistComponentData();

		int64_t now() const;

  	private:
	    std::fstream fp;

		ros::NodeHandle handle;

		std::map<std::string, std::vector<ComponentData>> componentMap;
		std::vector<std::string> components_in_cycle;
		std::vector<std::string> components_to_persist;

		ros::Publisher info_dataaccessnode2learning_pub;
		ros::Publisher info_dataaccessnode2controller_pub;

		std::string filepath;
		int logical_clock;
		bool first_persist;
};

#endif 