#ifndef MONITOR_MODULE_HPP
#define MONITOR_MODULE_HPP

#include <map>
#include <vector>
#include <math.h>  
#include <iostream>
#include <fstream>
#include <chrono>

#include "opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h"
#include "opendavinci/odcore/base/FIFOQueue.h"

#include "bsn/goalmodel/Task.hpp"
#include "bsn/goalmodel/Context.hpp"

#include "bsn/msg/info/MonitorTaskInfo.hpp"
#include "bsn/msg/info/MonitorContextInfo.hpp"

#include "bsn/time/TimeData.hpp"

#include "lepton/Lepton.h"

class MonitorModule : public odcore::base::module::TimeTriggeredConferenceClientModule {

    private:
      	MonitorModule(const MonitorModule &);
    	MonitorModule &operator=(const MonitorModule &);

    	virtual void setUp();
    	virtual void tearDown();

  public:
    	MonitorModule(const int32_t &argc, char **argv);
    	virtual ~MonitorModule();

    	odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

  	private:
		odcore::base::FIFOQueue buffer;

		std::map<std::string, bsn::goalmodel::Task> tasks;
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

		uint32_t persist;
		std::string path;
		std::ofstream fp;
};

#endif 