#ifndef G3T1_1_HPP
#define G3T1_1_HPP

#include <string>
#include <exception>

#include "ros/ros.h"

#include "bsn/resource/Battery.hpp"
#include "bsn/range/Range.hpp"
#include "bsn/generator/Markov.hpp"
#include "bsn/generator/DataGenerator.hpp"
#include "bsn/filters/MovingAverage.hpp"
#include "bsn/operation/Operation.hpp"
#include "bsn/configuration/SensorConfiguration.hpp"

#include "component/Sensor.hpp"

#include "messages/SensorData.h"

class G3T1_1 : public Sensor {

  	public:
		G3T1_1(int &argc, char **argv, const std::string &name);
    	~G3T1_1();

	private:
      	G3T1_1(const G3T1_1 &);
    	G3T1_1 &operator=(const G3T1_1 &);

	public:
    	void setUp();
    	void tearDown();

		bool check_failure(std::list<double> filter_buffer);

        double collect();
        double process(const double &data);
        void transfer(const double &data);

	  private:
		bsn::generator::Markov markov;
		bsn::filters::MovingAverage filter;
		bsn::configuration::SensorConfiguration sensorConfig;

		ros::NodeHandle handle;
		ros::Publisher data_pub;
};

#endif 