#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <stdio.h> 
#include <string>
#include <vector>

#include "archlib/target_system/Component.hpp"
#include "archlib/AdaptationCommand.h"
#include "archlib/Uncertainty.h"

#include "libbsn/resource/Battery.hpp"
#include "libbsn/utils/utils.hpp"

class Sensor : public arch::target_system::Component {

    public:
		Sensor(int &argc, char **argv, const std::string &name, const std::string &type, const bool &active, const double &noise_factor, const bsn::resource::Battery &battery, const bool &instant_recharge);
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
        bool instant_recharge;
        bool shouldStart;
        double cost;
};

#endif 