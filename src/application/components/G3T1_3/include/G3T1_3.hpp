#ifndef G3T1_3_HPP
#define G3T1_3_HPP

#include <fstream>
#include <chrono>
#include <string>
#include <iostream>

#include "ros/ros.h"

#include "bsn/range/Range.hpp"
#include "bsn/resource/Battery.hpp"
#include "bsn/generator/Markov.hpp"
#include "bsn/generator/DataGenerator.hpp"
#include "bsn/filters/MovingAverage.hpp"
#include "bsn/operation/Operation.hpp"
#include "bsn/configuration/SensorConfiguration.hpp"

#include "messages/SensorData.h"
#include "messages/Status.h"
#include "messages/Event.h"

class G3T1_3 {
    
	private:
      	G3T1_3(const G3T1_3 &);
    	G3T1_3 &operator=(const G3T1_3 &);
		
		void sendStatus(const std::string &/*id*/, const double &/*value*/);
		void sendEvent(const std::string &/*type*/, const std::string &/*description*/);

  	public:
    	virtual void setUp();
    	virtual void tearDown();

    	G3T1_3(const int32_t &argc, char **argv);
    	virtual ~G3T1_3();

    	void run();

  	private:
		std::string type;
		bsn::resource::Battery battery;
		bool available;

		double data_accuracy;
		double comm_accuracy;

		bool active;
		std::map<std::string, double> params;

		bsn::generator::Markov markov;
		bsn::filters::MovingAverage filter;
		bsn::configuration::SensorConfiguration sensorConfig;

		int persist;
		std::string path;
		std::ofstream fp;

		ros::Publisher status_pub, event_pub, dataPub;
};

#endif 