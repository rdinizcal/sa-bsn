#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP

#include <sstream>
#include <string>
#include <fstream>
#include <memory>
#include <map>

#include "ros/ros.h"
#include "ros/package.h"

#include "bsn/goalmodel/Node.hpp"
#include "bsn/goalmodel/Goal.hpp"
#include "bsn/goalmodel/Task.hpp"
#include "bsn/goalmodel/Property.hpp"
#include "bsn/goalmodel/LeafTask.hpp"
#include "bsn/goalmodel/Context.hpp"
#include "bsn/goalmodel/GoalTree.hpp"
#include "bsn/model/Formula.hpp"
#include "bsn/operation/Operation.hpp"

#include "lepton/Lepton.h"

//#include "messages/ControlCommand.h"
//#include "messages/SystemInfo.h"
//#include "messages/TaskInfo.h"
//#include "messages/ContextInfo.h"

class ControllerNode {

    private:
      	ControllerNode(const ControllerNode &);
    	ControllerNode &operator=(const ControllerNode &);

  	public:
      	void setUp();

		void setTaskValue(std::string, double);
		double getTaskValue(std::string);

		bool isCost(std::string);

		std::string getSensorName(const int);
		std::string getTaskActuator(std::string);
		std::string getContextActuator(std::string); 

    	ControllerNode(int &argc, char **argv, std::string name);
    	virtual ~ControllerNode();

		//void receiveTaskInfo(const messages::TaskInfo::ConstPtr& /*msg*/);
		//void receiveContextInfo(const messages::ContextInfo::ConstPtr& /*msg*/);

    	void analyze();
    	void plan();
    	void execute(double);

		void run();

  	private:

	std::map<std::string, double> tasks;
	std::map<std::string, bsn::goalmodel::Context> contexts;

	// May be redundant... should try to refactor it later
	std::vector<std::string> props;
	std::vector<double> values;

	double desired_reli;
	double desired_cost;

	double current_reli;
	double current_cost;
	
	double reli_error;
	double cost_error;

	bsn::model::Formula cost_expression;
	bsn::model::Formula reliability_expression;

	std::string message_id;

	// Ros publishers for the sensors
	ros::Publisher ecg_pub;
	ros::Publisher therm_pub;
	ros::Publisher oxi_pub;
	ros::Publisher abp_pub;
};

#endif 