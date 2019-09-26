#ifndef ENACTOR_HPP
#define ENACTOR_HPP

#include <map>
#include <deque>
#include <math.h>

#include "ros/ros.h"
#include "ros/package.h"

#include "archlib/Status.h"
#include "archlib/Event.h"
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

		void apply_strategy(const std::string &component);

		void print();

  	private:
		ros::Publisher adapt;

		std::map<std::string, double> reliability;
		std::map<std::string, std::deque<int>> executions; //a map of deques where 1s represent successes and 0s represents failures
		std::map<std::string, int> replicate_task;
		std::vector<std::string> connected;

		int64_t cycles;
};

#endif 