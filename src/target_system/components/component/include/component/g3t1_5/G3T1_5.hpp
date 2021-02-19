#ifndef G3T1_5_HPP
#define G3T1_5_HPP

#include <string>
#include <exception>

#include "ros/ros.h"

#include "libbsn/range/Range.hpp"
#include "libbsn/resource/Battery.hpp"
#include "libbsn/generator/Markov.hpp"
#include "libbsn/generator/DataGenerator.hpp"
#include "libbsn/filters/MovingAverage.hpp"
#include "libbsn/utils/utils.hpp"
#include "libbsn/configuration/SensorConfiguration.hpp"

#include "component/Sensor.hpp"

#include "messages/SensorData.h"
#include "services/PatientData.h"

class G3T1_5 : public Sensor {
    	
  	public:
    	G3T1_5(int &argc, char **argv, const std::string &name);
    	virtual ~G3T1_5();

	private:
      	G3T1_5(const G3T1_5 &);
    	G3T1_5 &operator=(const G3T1_5 &);

		std::string label(double &risk);
    
	public:
		virtual void setUp();
    	virtual void tearDown();

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