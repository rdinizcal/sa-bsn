#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <string>
#include <map>
#include <iostream>

#include "ros/ros.h"

#include "CurrentInfo.hpp"

#include "messages/ControllerInfo.h"
#include "services/ParameterReconfiguration.h"

#include "std_msgs/String.h"

class Controller {

	private:
		Controller(const Controller &);
		Controller &operator=(const Controller &);

		//virtual void tearDown();

	public:
		virtual void setUp();

		Controller(const int32_t &argc, char **argv, std::string);

		Controller();

		virtual ~Controller();


		void getInfo(const messages::ControllerInfo::ConstPtr& msg);

		CurrentInfo parseInfo(/*receiveInfo return type*/);

		int errorCalculation(int estimated_time);

		void reconfigureParameters();

		void adaptation(CurrentInfo module_info);

		void run();

	private:
		int setpoint; //Seconds

		std::map<std::string, CurrentInfo> info;
		std::map<std::string, float> new_parameters;

		ros::NodeHandle n;

		float adapt_frequency;

		bool empty;
};

#endif
