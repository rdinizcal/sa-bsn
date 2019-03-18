#ifndef ECG_MODULE_HPP
#define ECG_MODULE_HPP

#include <fstream>
#include <chrono>
#include <string>
#include <iostream>

#include "ros/ros.h"

#include "range/Range.hpp"
#include "resource/Battery.hpp"
#include "generator/Markov.hpp"
#include "filters/MovingAverage.hpp"
#include "operation/Operation.hpp"
#include "configuration/SensorConfiguration.hpp"

#include "bsn/SensorData.h"

class ECGModule {
    
	private:
      	ECGModule(const ECGModule &);
    	ECGModule &operator=(const ECGModule &);

    	virtual void tearDown();

		void sendTaskInfo(const std::string &/*task_id*/, const double &/*cost*/, const double &/*reliability*/, const double &/*frequency*/);
		void sendContextInfo(const std::string &/*context_id*/, const bool &/*value*/);

		void sendMonitorTaskInfo(const std::string &/*task_id*/, const double &/*cost*/, const double &/*reliability*/, const double &/*frequency*/);
		void sendMonitorContextInfo(const std::string &/*context_id*/, const bool &/*value*/);

  	public:
    	virtual void setUp();

    	ECGModule(const int32_t &argc, char **argv);
    	virtual ~ECGModule();

    	void run();

  	private:
		std::string type;
		bsn::resource::Battery battery;
		bool available;

		double data_accuracy;
		double comm_accuracy;


		bool active;
		std::map<std::string,double> params;

		bsn::generator::Markov markov;
		bsn::filters::MovingAverage filter;
		bsn::configuration::SensorConfiguration sensorConfig;

		int persist;
		std::string path;
		std::ofstream fp;
};

#endif 