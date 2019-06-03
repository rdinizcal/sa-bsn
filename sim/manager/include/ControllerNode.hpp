#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP

#include <sstream>
#include <string>
#include <fstream>
#include <map>

#include "ros/ros.h"

#include "goalmodel/Node.hpp"
#include "goalmodel/Goal.hpp"
#include "goalmodel/Task.hpp"
#include "goalmodel/Property.hpp"
#include "goalmodel/LeafTask.hpp"
#include "goalmodel/Context.hpp"
#include "goalmodel/GoalTree.hpp"

#include "lepton/Lepton.h"

#include "model/Formula.hpp"

#include "bsn/TaskInfo.h"
#include "bsn/ControlCommand.h"
#include "bsn/SystemInfo.h"
#include "bsn/ContextInfo.h"

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

		void receiveTaskInfo(const bsn::TaskInfo::ConstPtr& /*msg*/);
		void receiveContextInfo(const bsn::ContextInfo::ConstPtr& /*msg*/);

    	void analyze(std::string);
    	void plan(std::string, double, double);
    	void execute(std::string, double, double, double);

		void run();

  	private:

	std::map<std::string, double> tasks;
	std::map<std::string, bsn::goalmodel::Context> contexts;

	// May be redundant... should try to refactor it later
	std::vector<std::string> props;
	std::vector<double> values;

	double reli_value;
	double cost_value;
	double reli_error;
	double cost_error;

	bsn::model::Formula cost_expression;
	bsn::model::Formula reliability_expression;

//	ros::Publisher actuator_pub;
	ros::Publisher centralhub_pub;
	ros::Publisher ecg_pub;
	ros::Publisher therm_pub;
	ros::Publisher oxi_pub;
	ros::Publisher abp_pub;
};

#endif 