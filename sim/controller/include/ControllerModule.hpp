#ifndef MANAGER_MODULE_HPP
#define MANAGER_MODULE_HPP

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

#include "bsn/msg/info/TaskInfo.hpp"
#include "bsn/msg/info/ContextInfo.hpp"
#include "bsn/msg/info/PatientStatusInfo.h"

#include "bsn/msg/control/ThermometerControlCommand.hpp"
#include "bsn/msg/control/OximeterControlCommand.hpp"
#include "bsn/msg/control/ECGControlCommand.hpp"
#include "bsn/msg/control/BloodpressureControlCommand.hpp"
#include "bsn/msg/control/CentralHubControlCommand.hpp"

#include "bsn/time/TimeData.hpp"

#include "lepton/Lepton.h"

class ControllerModule : public odcore::base::module::TimeTriggeredConferenceClientModule {

    private:
      	ControllerModule(const ControllerModule &);
    	ControllerModule &operator=(const ControllerModule &);

    	virtual void setUp();
    	virtual void tearDown();

  public:
    	ControllerModule(const int32_t &argc, char **argv);
    	virtual ~ControllerModule();

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

		std::vector<std::vector<double>> actions;

		uint32_t persist;
		std::string path;
		std::ofstream fp;
};

#endif 