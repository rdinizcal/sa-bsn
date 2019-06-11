#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP

#include <sstream>
#include <string>
#include <fstream>
#include <map>
#include <memory>

#include "goalmodel/Node.hpp"
#include "goalmodel/Goal.hpp"
#include "goalmodel/Task.hpp"
#include "goalmodel/Property.hpp"
#include "goalmodel/LeafTask.hpp"
#include "goalmodel/Context.hpp"
#include "goalmodel/GoalTree.hpp"

#include "lepton/Lepton.h"

#include "model/Formula.hpp"

#include <cpprest/http_client.h>
#include <cpprest/json.h>

#include "ros/ros.h"
#include <ros/package.h>

#include "bsn/TaskInfo.h"
#include "bsn/ControlCommand.h"
#include "bsn/SystemInfo.h"
#include "bsn/ContextInfo.h"

#include "operation/Operation.hpp"

class AnalyticsNode {

    private:
      	AnalyticsNode(const AnalyticsNode &);
    	AnalyticsNode &operator=(const AnalyticsNode &);

  	public:
      	void setUp();

		void setTaskValue(std::string, double);
		double getTaskValue(std::string);

		bool isCost(std::string);
/*

		std::string getSensorName(const int);
		std::string getTaskActuator(std::string);
		std::string getContextActuator(std::string); 
*/

		void receiveTaskInfo(const bsn::TaskInfo::ConstPtr& /*msg*/);
		void receiveContextInfo(const bsn::ContextInfo::ConstPtr& /*msg*/);

    	AnalyticsNode(int &argc, char **argv, std::string);
    	virtual ~AnalyticsNode();

		void analyze();
        void sendToServer();

		void run();

  	private:

		int session;
		bool connect;

		std::string database_url;
		std::string path;

		double current_reli;
		double current_cost;

		std::map<std::string, double> tasks;
		std::map<std::string, bsn::goalmodel::Context> contexts;

	// May be redundant... should try to refactor it later
		std::vector<std::string> props;
		std::vector<double> values;

		double desired_reli;
		double desired_cost;
		
		double reli_error;
		double cost_error;

		bsn::model::Formula cost_expression;
		bsn::model::Formula reliability_expression;
		
		std::string message_id;
/*

//	ros::Publisher actuator_pub;

	ros::Publisher centralhub_pub;
	ros::Publisher ecg_pub;
	ros::Publisher therm_pub;
	ros::Publisher oxi_pub;
	ros::Publisher abp_pub;
*/
};

#endif 