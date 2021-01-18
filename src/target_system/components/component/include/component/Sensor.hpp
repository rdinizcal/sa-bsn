#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <stdio.h> 
#include <string>
#include <vector>
#include <chrono>
#include <fstream>

#include <ros/package.h>

#include "archlib/target_system/Component.hpp"
#include "archlib/AdaptationCommand.h"
#include "archlib/Uncertainty.h"
#include "messages/SensorFrequency.h"
#include "messages/DiagnosticsData.h"

#include "bsn/resource/Battery.hpp"
#include "bsn/utils/utils.hpp"

#include "boost/date_time/posix_time/posix_time.hpp"

class Sensor : public arch::target_system::Component {

    public:
		Sensor(int &argc, char **argv, const std::string &name, const std::string &type, const bool &active, const double &noise_factor, const bsn::resource::Battery &battery);
    	~Sensor();

	private:
    	Sensor &operator=(const Sensor &);

  	public:
        virtual void setUp() = 0;
    	virtual void tearDown() = 0;
        virtual int32_t run();
		void body();
        void apply_noise(double &data);

        void reconfigure(const archlib::AdaptationCommand::ConstPtr& msg);
        void injectUncertainty(const archlib::Uncertainty::ConstPtr& msg);
        int64_t now() const;
        void flushData(messages::DiagnosticsData msg);
		
        virtual double collect() = 0;
        virtual double process(const double &data) = 0;
        virtual void transfer(const double &data) = 0;

    protected:
        bool isActive();
        void turnOn();
        void turnOff();
        void recharge();
        
    protected:
		std::string type;
		bool active;
        int buffer_size;
        int replicate_collect;
		double noise_factor;
		bsn::resource::Battery battery;
        double data;

        ros::Publisher frequency_pub;
        ros::Publisher statusPub;
        ros::Publisher detectedPub;

        uint32_t dataId;

        std::string filepath;
        std::string timestamp; 
        std::fstream fp;
        std::string currentProperty;
        std::string pathSuffix;
        boost::posix_time::ptime my_posix_time;
};

#endif 