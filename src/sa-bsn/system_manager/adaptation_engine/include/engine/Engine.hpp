#ifndef ENGINE_HPP
#define ENGINE_HPP

#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <set>
#include <algorithm>

#include "ros/ros.h"
#include "ros/package.h"

#include "libbsn/goalmodel/Node.hpp"
#include "libbsn/goalmodel/Goal.hpp"
#include "libbsn/goalmodel/Task.hpp"
#include "libbsn/goalmodel/Property.hpp"
#include "libbsn/goalmodel/LeafTask.hpp"
#include "libbsn/goalmodel/Context.hpp"
#include "libbsn/goalmodel/GoalTree.hpp"
#include "libbsn/model/Formula.hpp"
#include "libbsn/utils/utils.hpp"

#include "lepton/Lepton.h"

#include "archlib/DataAccessRequest.h"
#include "archlib/Strategy.h"
#include "archlib/Exception.h"
#include "archlib/EnergyStatus.h"
#include "archlib/ROSComponent.hpp"
#include "archlib/EngineRequest.h"


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
		bool sendAdaptationParameter(archlib::EngineRequest::Request &req, archlib::EngineRequest::Response &res);

		virtual std::string get_prefix() = 0;
		virtual std::map<std::string, int> initialize_priority(std::vector<std::string>) = 0;
		virtual std::map<std::string, double> initialize_strategy(std::vector<std::string>) = 0;
		std::string fetch_formula(std::string);
		void setUp_formula(std::string formula);

	  	double calculate_qos(bsn::model::Formula, std::map<std::string, double>);
	  	bool blacklisted(std::map<std::string,double> &);

		virtual void monitor() = 0;
    	virtual void analyze() = 0;
    	virtual void plan()    = 0;
    	virtual void execute() = 0;

	private: 
		std::string qos_attribute;

	public:
		double info_quant;
		double monitor_freq;
		double actuation_freq;

		bsn::model::Formula target_system_model;
		std::map<std::string, double> strategy;
		std::map<std::string, int> priority;
		std::map<std::string, int> deactivatedComponents;

		ros::ServiceServer enactor_server;
};

#endif 