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
#include "bsn/utils/utils.hpp"
#include "bsn/configuration/SensorConfiguration.hpp"

#include "component/Sensor.hpp"

#include "services/PatientData.h"
#include "messages/SensorData.h"

class G3T1_1 : public Sensor {

  	public:
		G3T1_1(int &argc, char **argv, const std::string &name);
    	~G3T1_1();

	private:
      	G3T1_1(const G3T1_1 &);
    	G3T1_1 &operator=(const G3T1_1 &);

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
		
		double collected_risk;
		uint32_t msg_id;
};

#endif 