#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <fstream>
#include <chrono>

#include "ros/ros.h"
#include <ros/package.h>

#include "archlib/Status.h"
#include "archlib/EnergyStatus.h"
#include "archlib/Event.h"
#include "archlib/AdaptationCommand.h"
#include "archlib/Uncertainty.h"
#include "archlib/Persist.h"

#include "archlib/ROSComponent.hpp"

class Logger : public arch::ROSComponent {

	public:
    	Logger(int &argc, char **argv, const std::string &name);
    	virtual ~Logger();

    private:
      	Logger(const Logger &);
    	Logger &operator=(const Logger &);
		int64_t now() const;

	public:
		virtual void setUp();
		virtual void tearDown();
		virtual void body();

	  	void receiveAdaptationCommand(const archlib::AdaptationCommand::ConstPtr& msg);
	  	void receiveStatus(const archlib::Status::ConstPtr& msg);
		void receiveEnergyStatus(const archlib::EnergyStatus::ConstPtr& msg);
	  	void receiveEvent(const archlib::Event::ConstPtr& msg);
	  	void receiveUncertainty(const archlib::Uncertainty::ConstPtr& msg);

  	protected:
        ros::NodeHandle handle;

	private:
		int64_t time_ref;
		ros::Publisher adapt, status, event, persist;
};

#endif 