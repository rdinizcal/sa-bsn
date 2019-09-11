#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <stdio.h> 
#include <string>

#include "archlib/target_system/Component.hpp"
#include "archlib/AdaptationCommand.h"

#include "bsn/resource/Battery.hpp"

class Sensor : public arch::target_system::Component {

    public:
		Sensor(int &argc, char **argv, const std::string &name, const std::string &type, const bool &active, const double &accuracy, const bsn::resource::Battery &battery);
    	~Sensor();

	private:
    	Sensor &operator=(const Sensor &);

  	public:
        virtual void setUp() = 0;
    	virtual void tearDown() = 0;
		void body();

        void reconfigure(const archlib::AdaptationCommand::ConstPtr& msg);
		
        virtual double collect();
        virtual double process(const double &data);
        virtual void transfer(const double &data);

    private:
        bool isActive();
        void turnOn();
        void turnOff();
        void recharge();

    protected:
		std::string type;
		bool active;
		double accuracy;
		bsn::resource::Battery battery;
        double data;
    
};

#endif 