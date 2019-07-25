#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <fstream>
#include <iostream>
#include <chrono>

#include "ros/ros.h"
#include <ros/package.h>

#include "messages/TaskInfo.h"
#include "messages/ContextInfo.h"

class Logger {

	public:
    	Logger(int &argc, char **argv, std::string);
    	virtual ~Logger();

		void setUp();
		void run();

    private:
      	Logger(const Logger &);
    	Logger &operator=(const Logger &);

		std::string now() const;

  	public:
		void receiveTaskInfo(const messages::TaskInfo::ConstPtr& /*msg*/);
		void receiveContextInfo(const messages::ContextInfo::ConstPtr& /*msg*/);


  	private:
		std::fstream fp;
		std::string filepath;
		int32_t logical_clock;

		ros::Publisher task_logger_manager_pub;
		ros::Publisher context_logger_manager_pub;

};

#endif 