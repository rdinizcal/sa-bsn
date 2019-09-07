#ifndef G3T1_2_HPP
#define G3T1_2_HPP

#include <string>
#include <exception>

#include "ros/ros.h"

#include "bsn/generator/DataGenerator.hpp"
#include "bsn/range/Range.hpp"
#include "bsn/resource/Battery.hpp"
#include "bsn/generator/Markov.hpp"
#include "bsn/filters/MovingAverage.hpp"
#include "bsn/operation/Operation.hpp"
#include "bsn/configuration/SensorConfiguration.hpp"

#include "component/Sensor.hpp"

#include "messages/SensorData.h"
#include "messages/Info.h"

class G3T1_2 : public Sensor {
    
	private:
      	G3T1_2(const G3T1_2 &);
    	G3T1_2 &operator=(const G3T1_2 &);

  	public:
	    G3T1_2(const int32_t &argc, char **argv);
    	~G3T1_2();

    	void setUp();
    	void tearDown();

        double collect();
        double process(const double &data);
        void transfer(const double &data);

  	private:
		bsn::generator::Markov markov;
		bsn::filters::MovingAverage filter;
		bsn::configuration::SensorConfiguration sensorConfig;

		ros::NodeHandle handle;
		ros::Publisher data_pub, info_pub;
};

#endif 