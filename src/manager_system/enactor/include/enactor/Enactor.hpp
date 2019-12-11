#ifndef ENACTOR_HPP
#define ENACTOR_HPP

#include <map>
#include <deque>
#include <math.h>
#include <numeric>

#include "ros/ros.h"
#include "ros/package.h"

#include "bsn/operation/Operation.hpp"

#include "archlib/Status.h"
#include "archlib/Event.h"
#include "archlib/Strategy.h"
#include "archlib/Exception.h"
#include "archlib/AdaptationCommand.h"

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

	  	void receiveStatus(const archlib::Status::ConstPtr& msg);
	  	void receiveEvent(const archlib::Event::ConstPtr& msg);
	  	void receiveStrategy(const archlib::Strategy::ConstPtr& msg);

		void apply_strategy(const std::string &component);

		void print();

  	private:
		ros::Publisher adapt;
		ros::Publisher except;

		float Kp;
		float Ki;
		int IW;
		std::map<std::string, std::vector<double>> error_window;
		
		std::map<std::string, double> r_curr;
		std::map<std::string, double> r_ref;
		//std::map<std::string, double> kp;
		std::map<std::string, std::deque<int>> invocations; //a map of deques where 1s represent successes and 0s represents failures
		std::map<std::string, int> replicate_task;
		std::map<std::string, double> freq;
		std::map<std::string, int> exception_buffer;

		int64_t cycles;
		double stability_margin;
};

#endif 