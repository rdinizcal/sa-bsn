#ifndef ENACTOR_HPP
#define ENACTOR_HPP

#include <map>
#include <deque>
#include <math.h>

#include "ros/ros.h"
#include "ros/package.h"

#include "services/EnactorInfo.h"

#include "bsn/utils/utils.hpp"

#include "archlib/Status.h"
#include "archlib/Event.h"
#include "archlib/Strategy.h"
#include "archlib/Exception.h"
#include "archlib/AdaptationCommand.h"

#include "archlib/DataAccessRequest.h"
#include "archlib/ROSComponent.hpp"

class Enactor : public arch::ROSComponent {

    public:
    	Enactor(int &argc, char **argv, std::string name);
    	virtual ~Enactor();

    private:
      	Enactor(const Enactor &);
    	Enactor &operator=(const Enactor &);

  	public:
        virtual void setUp();
    	virtual void tearDown();
		virtual void body();

	  	void receiveStatus();
		void receiveEvent(const archlib::Event::ConstPtr& msg);
	  	void receiveStrategy(const archlib::Strategy::ConstPtr& msg);
		bool sendInfo(services::EnactorInfo::Request &req, services::EnactorInfo::Response &res);

		void apply_strategy(const std::string &component);

		void print();

  	private:

	  	float Kp;

	  	ros::NodeHandle handle;
		ros::Publisher adapt;
		ros::Publisher except;
		ros::ServiceServer enactor_info_server;

		std::map<std::string, double> r_curr;
		std::map<std::string, double> r_ref;
		std::map<std::string, double> kp;
		std::map<std::string, std::deque<int>> invocations; //a map of deques where 1s represent successes and 0s represents failures
		std::map<std::string, int> replicate_task;
		std::map<std::string, double> freq;
		std::map<std::string, int> exception_buffer;

		int64_t cycles;
		double stability_margin;
};

#endif 