#ifndef ENACTOR_HPP
#define ENACTOR_HPP

#include <map>
#include <deque>
#include <math.h>

#include "ros/ros.h"
#include "ros/package.h"

#include "bsn/operation/Operation.hpp"

#include "archlib/Status.h"
#include "archlib/Event.h"
#include "archlib/Strategy.h"
#include "archlib/Exception.h"
#include "archlib/AdaptationCommand.h"

#include "archlib/ROSComponent.hpp"
#include "archlib/system_manager/StrategyEnactor.hpp"


class Enactor : public archlib::system_manager::StrategyEnactor {

    public:
    	Enactor(int &argc, char **argv, const std::string &name);
    	~Enactor();

    private:
      	Enactor(const Enactor &);
    	Enactor &operator=(const Enactor &);

  	public:
        void setUp() override;
    	void tearDown() override;
		void body() override;

	  	void receiveStatus(const archlib::Status::ConstPtr& msg) override;
	  	void receiveEvent(const archlib::Event::ConstPtr& msg) override;
	  	void receiveStrategy(const archlib::Strategy::ConstPtr& msg) override;

		void apply_strategy(const std::string &component) override;

		void print();

  	private:
		ros::Publisher adapt;
		ros::Publisher except;

		std::map<std::string, double> r_curr;
		std::map<std::string, double> r_ref;
		std::map<std::string, double> kp;
		std::map<std::string, std::deque<int>> invocations; //a map of deques where 1s represent successes and 0s represents failures
		std::map<std::string, int> replicate_task;
		std::map<std::string, double> freq;
		std::map<std::string, int> exception_buffer;

		ros::Subscriber subs_event;
		ros::Subscriber subs_status;
		ros::Subscriber subs_strategy;

		int64_t cycles;
		double stability_margin;
};

#endif 