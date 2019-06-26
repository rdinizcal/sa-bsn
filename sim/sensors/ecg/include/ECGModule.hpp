#ifndef ECG_MODULE_HPP
#define ECG_MODULE_HPP

#include <fstream>
#include <chrono>
#include <string>
#include <iostream>

#include "ros/ros.h"

#include "bsn/generator/DataGenerator.hpp"
#include "bsn/range/Range.hpp"
#include "bsn/resource/Battery.hpp"
#include "bsn/generator/Markov.hpp"
#include "bsn/filters/MovingAverage.hpp"
#include "bsn/operation/Operation.hpp"
#include "bsn/configuration/SensorConfiguration.hpp"

#include "messages/SensorData.h"
#include "messages/TaskInfo.h"
#include "messages/ContextInfo.h"
#include "messages/ControlCommand.h"

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
		void receiveControlCommand(const messages::ControlCommand::ConstPtr& msg);

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

		ros::Publisher taskPub, contextPub, dataPub;
};

class ECGCollect {

	private:
      	ECGCollect(const ECGCollect &);		
    	ECGCollect &operator=(const ECGCollect &);

	public:
		ECGCollect(const int32_t &argc, char **argv);
    	virtual ~ECGCollect();

		double collect(void);
		
	private:
		bsn::generator::Markov markov;
		bsn::resource::Battery battery;
	
}

class ECGFilter {
    
	private:
      	ECGFilter(const ECGFilter &);
    	ECGFilter &operator=(const ECGFilter &);

	public:
		ECGFilter(const int32_t &argc, char **argv);
		virtual ~ECGFilter(void);

		double filter(double);
    
	private:
		bsn::resource::Battery battery;
		bsn::filters::MovingAverage filter;
}

class ECGTransfer {

	private:
      	ECGTransfer(const ECGTransfer &);
    	ECGTransfer &operator=(const ECGTransfer &);

	public:
		ECGTransfer(const int32_t &argc, char **argv);
		virtual ~ECGTransfer(void);

		void transfer(double);

	private:
		bsn::resource::Battery battery;
		ros::Publisher dataPub;

}


#endif 