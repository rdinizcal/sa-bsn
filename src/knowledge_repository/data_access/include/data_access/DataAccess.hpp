#ifndef DATA_ACCESS_HPP
#define DATA_ACCESS_HPP

#include <fstream>
#include <chrono>

#include "ros/ros.h"
#include <ros/package.h>

#include "archlib/Persist.h"
#include "archlib/ROSComponent.hpp"

#include "StatusMessage.hpp"
#include "EventMessage.hpp"
#include "UncertaintyMessage.hpp"

class DataAccess : public arch::ROSComponent {

	public:
		DataAccess(int &argc, char **argv, const std::string &name);
		virtual ~DataAccess();

	private:
		DataAccess(const DataAccess &);
		DataAccess &operator=(const DataAccess &);
		int64_t now() const;

		void persistEvent(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content);
		void persistStatus(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content);
		void persistUncertainty(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content);

		void flush();

	public:
		virtual void setUp();
		virtual void tearDown();
		virtual void body();

		void receivePersistMessage(const archlib::Persist::ConstPtr& msg);

	protected:
		ros::NodeHandle handle;
	
	private:
		std::fstream fp;
		std::string event_filepath;
		std::string status_filepath;
		std::string  uncertainty_filepath;

		int64_t logical_clock;

		std::vector<StatusMessage> statusVec;
		std::vector<EventMessage> eventVec;
		std::vector<UncertaintyMessage> uncertainVec;

};

#endif 