#ifndef ENGINE_HPP
#define ENGINE_HPP

#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <set>
#include <algorithm>
#include <chrono>

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
#include "bsn/utils/utils.hpp"

#include "lepton/Lepton.h"

#include "archlib/DataAccessRequest.h"
#include "archlib/Strategy.h"
#include "archlib/Exception.h"
#include "archlib/ROSComponent.hpp"
#include "archlib/Persist.h"

class Engine : public arch::ROSComponent {
	
	public: 
		Engine(int &argc, char **argv, std::string name);
    	virtual ~Engine();

    private:
      	Engine(const Engine &);
    	Engine &operator=(const Engine &);

  	public:
        virtual void setUp();
    	virtual void tearDown();
		virtual void body();

		void receiveException(const archlib::Exception::ConstPtr& msg);

		void monitor();
    	void analyze();
    	void plan();
    	void execute();


  	private:
	  double calculate_reli();
	  bool blacklisted(std::map<std::string,double> &);

	private:
		double r_ref;
		double offset;
		double Kp;
		double info_quant;
		double monitor_freq;
		double actuation_freq;
		double stability_margin;

		bsn::model::Formula reliability_expression;
		std::map<std::string, double> strategy;
		std::map<std::string, int> priority;

		ros::NodeHandle handle;
		ros::Publisher enact;
		ros::Publisher persist_pub;

		int cycles;
		int counter;
};

#endif 