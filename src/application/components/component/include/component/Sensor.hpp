#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <exception>

#include "ros/ros.h"

#include "bsn/resource/Battery.hpp"

#include "SchedulableComponent.hpp"

#include "messages/Status.h"
#include "messages/Event.h"

class Sensor : public SchedulableComponent {

    public:
		Sensor(const int32_t &argc, char **argv, const std::string &type, const bool &active, const double &accuracy, const bsn::resource::Battery &battery);
    	~Sensor();

	private:
    	Sensor &operator=(const Sensor &);

  	public:
		void sendEvent(const std::string &/*type*/, const std::string &/*description*/);
		void sendStatus(const std::string &/*key*/, const double &/*value*/);

        virtual void setUp() = 0;
    	virtual void tearDown() = 0;
		void body();

        virtual double collect();
        virtual double process(const double &data);
        virtual void transfer(const double &data);

    private:
        bool isActive();
        void turnOn();
        void turnOff();
        void recharge();

	private:
		ros::Publisher status_pub, event_pub;
		ros::NodeHandle handle;

    protected:
		std::string type;
		bool active;
		double accuracy;
		bsn::resource::Battery battery;
        double data;
    
};

#endif 