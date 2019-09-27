#ifndef G3T1_4_HPP
#define G3T1_4_HPP

#include <string>
#include <exception>

#include "ros/ros.h"

#include "bsn/range/Range.hpp"
#include "bsn/resource/Battery.hpp"
#include "bsn/generator/Markov.hpp"
#include "bsn/generator/DataGenerator.hpp"
#include "bsn/filters/MovingAverage.hpp"
#include "bsn/operation/Operation.hpp"
#include "bsn/configuration/SensorConfiguration.hpp"

#include "component/Sensor.hpp"

#include "messages/SensorData.h"

class G3T1_4 : public Sensor {
    
    
  	public:
    	G3T1_4(int &argc, char **argv, const std::string &name);
    	~G3T1_4();

	private:
      	G3T1_4(const G3T1_4 &);
    	G3T1_4 &operator=(const G3T1_4 &);

		std::string label(bsn::configuration::SensorConfiguration &sensorConfig, double &risk);
	
	public:
		void setUp();
    	void tearDown();
		void body();
		
		double collect();
        double process(const double &data);
        void transfer(const double &data);

	private:
		double collectSystolic();
		double collectDiastolic();

        double processSystolic(const double &data);
        double processDiastolic(const double &data);

        void transferSystolic(const double &data);
        void transferDiastolic(const double &data);


  	private:
		bsn::generator::Markov markovSystolic;
		bsn::generator::DataGenerator dataGeneratorSystolic;
		bsn::generator::Markov markovDiastolic;
		bsn::generator::DataGenerator dataGeneratorDiastolic;
		bsn::filters::MovingAverage filterSystolic;
		bsn::filters::MovingAverage filterDiastolic;
		bsn::configuration::SensorConfiguration sensorConfigSystolic;
		bsn::configuration::SensorConfiguration sensorConfigDiastolic;
		double systolic_data;
		double diastolic_data;

		ros::NodeHandle handle;
		ros::Publisher data_pub;

		double collected_systolic_risk;
		double collected_diastolic_risk;

};

#endif 