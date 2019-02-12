#ifndef BLOODPRESSURE_MODULE_HPP
#define BLOODPRESSURE_MODULE_HPP

#include <fstream>
#include <chrono>

#include "opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h"
#include "opendavinci/odcore/base/FIFOQueue.h"

#include "bsn/range/Range.hpp"
#include "bsn/resource/Battery.hpp"
#include "bsn/generator/Markov.hpp"
#include "bsn/filters/MovingAverage.hpp"
#include "bsn/operation/Operation.hpp"
#include "bsn/configuration/SensorConfiguration.hpp"

#include "bsn/msg/data/SensorData.h"
#include "bsn/msg/info/TaskInfo.hpp"
#include "bsn/msg/info/ContextInfo.hpp"
#include "bsn/msg/info/MonitorTaskInfo.hpp"
#include "bsn/msg/info/MonitorContextInfo.hpp"
#include "bsn/msg/control/BloodpressureControlCommand.hpp"

class BloodpressureModule : public odcore::base::module::TimeTriggeredConferenceClientModule{
    
	private:
      	BloodpressureModule(const BloodpressureModule &);
    	BloodpressureModule &operator=(const BloodpressureModule &);
    	
		virtual void setUp();
    	virtual void tearDown();

		void sendTaskInfo(const std::string &/*task_id*/, const double &/*cost*/, const double &/*reliability*/, const double &/*frequency*/);
		void sendContextInfo(const std::string &/*context_id*/, const bool &/*value*/);

		void sendMonitorTaskInfo(const std::string &/*task_id*/, const double &/*cost*/, const double &/*reliability*/, const double &/*frequency*/);
		void sendMonitorContextInfo(const std::string &/*context_id*/, const bool &/*value*/);

  	public:
    	BloodpressureModule(const int32_t &argc, char **argv);
    	virtual ~BloodpressureModule();

    	odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

  	private:
	    odcore::base::FIFOQueue buffer;
		
		std::string type;
		bsn::resource::Battery battery;
		bool available;

		double diasdata_accuracy;
		double diascomm_accuracy;
		double systdata_accuracy;
		double systcomm_accuracy;


		bool active;
		std::map<std::string,double> params;

		bsn::generator::Markov markovSystolic;
		bsn::generator::Markov markovDiastolic;
		bsn::filters::MovingAverage filterSystolic;
		bsn::filters::MovingAverage filterDiastolic;
		bsn::configuration::SensorConfiguration sensorConfigSystolic;
		bsn::configuration::SensorConfiguration sensorConfigDiastolic;

		int persist;
		std::string path;
		std::ofstream fp;
};

#endif 