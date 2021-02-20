#ifndef G3T1_2_HPP
#define G3T1_2_HPP

#include <string>
#include <exception>

#include "ros/ros.h"

#include "libbsn/generator/DataGenerator.hpp"
#include "libbsn/range/Range.hpp"
#include "libbsn/resource/Battery.hpp"
#include "libbsn/generator/Markov.hpp"
#include "libbsn/filters/MovingAverage.hpp"
#include "libbsn/utils/utils.hpp"
#include "libbsn/configuration/SensorConfiguration.hpp"

#include "component/Sensor.hpp"

#include "messages/SensorData.h"
#include "services/PatientData.h"

class G3T1_2 : public Sensor {
    
  	public:
	    G3T1_2(int &argc, char **argv, const std::string &name);
    	~G3T1_2();

	private:
      	G3T1_2(const G3T1_2 &);
    	G3T1_2 &operator=(const G3T1_2 &);

		std::string label(double &risk);

	public:
    	void setUp();
    	void tearDown();

        double collect();
        double process(const double &data);
        void transfer(const double &data);

  	private:
		bsn::generator::Markov markov;
		bsn::generator::DataGenerator dataGenerator;
		bsn::filters::MovingAverage filter;
		bsn::configuration::SensorConfiguration sensorConfig;

		ros::NodeHandle handle;
		ros::Publisher data_pub;
		ros::ServiceClient client;

		double collected_risk;

};

#endif 