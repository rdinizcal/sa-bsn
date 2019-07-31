#ifndef EFFECTOR_HPP
#define EFFECTOR_HPP

#include <string>

#include "ros/ros.h"

#include "services/SchedulerServerData.h"
#include "messages/ReconfigurationCommand.h"

class Effector {

	public:
    	Effector(int &argc, char **argv);
    	virtual ~Effector();

		void setUp();
		void run();

    private:
      	Effector(const Effector &);
    	Effector &operator=(const Effector &);

  	public:
		void receiveReconfigurationCommand(const messages::ReconfigurationCommand::ConstPtr& /*msg*/);
		bool moduleConnect(services::SchedulerServerData::Request &req, services::SchedulerServerData::Response &res);

  	private:
		ros::NodeHandle handler;
		ros::ServiceServer register_service;
	  	std::map<std::string,ros::Publisher> reconfigure_pub;

};

#endif 