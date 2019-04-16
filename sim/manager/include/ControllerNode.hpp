#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP

#include <sstream>
#include <string>
#include <fstream>

#include "ros/ros.h"

#include "goalmodel/Node.hpp"
#include "goalmodel/Goal.hpp"
#include "goalmodel/Task.hpp"
#include "goalmodel/Property.hpp"
#include "goalmodel/LeafTask.hpp"
#include "goalmodel/Context.hpp"
#include "goalmodel/GoalTree.hpp"

#include "Lepton.h"

#include "FormulaRefs.hpp"

#include "bsn/TaskInfo.h"
#include "bsn/ContextInfo.h"

class ControllerNode {

    private:
      	ControllerNode(const ControllerNode &);
    	ControllerNode &operator=(const ControllerNode &);

  public:
      	void setUp();

    	ControllerNode(int &argc, char **argv, std::string name);
    	virtual ~ControllerNode();

		void receiveTaskInfo(const bsn::TaskInfo::ConstPtr& /*msg*/);
		void receiveContextInfo(const bsn::ContextInfo::ConstPtr& /*msg*/);

    	void analyze();
    	void plan();
    	void execute();

		void run();

  	private:

	std::vector<bsn::goalmodel::LeafTask> tasks;
	std::map<std::string, bsn::goalmodel::Context> contexts;

	Lepton::CompiledExpression cost_expression;
	Lepton::CompiledExpression reliability_expression;

	std::map<std::string,double&> cost_formula_reliabilities;
	std::map<std::string,double&> cost_formula_frequencies;
	std::map<std::string,double&> cost_formula_costs;
	std::map<std::string,double&> cost_formula_contexts;

	std::map<std::string,double&> reliability_formula_reliabilities;
	std::map<std::string,double&> reliability_formula_frequencies;
	std::map<std::string,double&> reliability_formula_contexts;

	std::vector<std::vector<double>> actions;

	double cost_setpoint;
	double reliability_setpoint;
};

#endif 