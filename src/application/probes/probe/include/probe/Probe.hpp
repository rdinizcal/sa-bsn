#ifndef PROBE_HPP
#define PROBE_HPP

#include <fstream>
#include <iostream>
#include <chrono>

#include "ros/ros.h"

#include "messages/Status.h"
#include "messages/Event.h"
#include "messages/Info.h"

class Probe {

	public:
    	Probe(int &argc, char **argv);
    	virtual ~Probe();

		void setUp();
		void run();

    private:
      	Probe(const Probe &);
    	Probe &operator=(const Probe &);

  	public:
		void receiveStatus(const messages::Status::ConstPtr& /*msg*/);
		void receiveEvent(const messages::Event::ConstPtr& /*msg*/);
		void receiveInfo(const messages::Info::ConstPtr& /*msg*/);

  	private:
	  ros::Publisher status_pub, event_pub, info_pub;
};

#endif 