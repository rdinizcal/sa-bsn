#ifndef DATA_ACCESS_HPP
#define DATA_ACCESS_HPP

#include <fstream>
#include <chrono>
#include <deque>

#include "ros/ros.h"
#include <ros/package.h>
#include <cpprest/http_client.h>
#include <cpprest/json.h>

#include "bsn/goalmodel/Node.hpp"
#include "bsn/goalmodel/Goal.hpp"
#include "bsn/goalmodel/Task.hpp"
#include "bsn/goalmodel/Property.hpp"
#include "bsn/goalmodel/LeafTask.hpp"
#include "bsn/goalmodel/Context.hpp"
#include "bsn/goalmodel/GoalTree.hpp"
#include "bsn/model/Formula.hpp"
#include "bsn/operation/Operation.hpp"

#include "archlib/Persist.h"
#include "archlib/DataAccessRequest.h"
#include "archlib/ROSComponent.hpp"

#include "StatusMessage.hpp"
#include "EventMessage.hpp"
#include "UncertaintyMessage.hpp"
#include "AdaptationMessage.hpp"

#include "lepton/Lepton.h"

#include "messages/TargetSystemData.h"

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
		void persistAdaptation(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content);


		void flush();

		double calculateCost();
		double calculateReliability();

	public:
		virtual void setUp();
		virtual void tearDown();
		virtual void body();

		void processTargetSystemData(const messages::TargetSystemData::ConstPtr &msg);
		void receivePersistMessage(const archlib::Persist::ConstPtr& msg);
		bool processQuery(archlib::DataAccessRequest::Request &req, archlib::DataAccessRequest::Response &res);

	protected:
		ros::NodeHandle handle;
	
	private:
		std::fstream fp;
		std::string event_filepath;
		std::string status_filepath;
		std::string uncertainty_filepath;
		std::string adaptation_filepath;

		int64_t logical_clock;

		std::vector<StatusMessage> statusVec;
		std::vector<EventMessage> eventVec;
		std::vector<UncertaintyMessage> uncertainVec;
		std::vector<AdaptationMessage> adaptVec;

		std::map<std::string, std::deque<std::string>> status;
		std::map<std::string, std::deque<std::string>> events;
		int buffer_size;

		std::map<std::string, double> componentsReliabilities;
		std::map<std::string, double> componentsBatteries;
		std::map<std::string, double> componentsCosts;
		std::map<std::string, uint32_t> contexts;

		bsn::model::Formula reliability_expression;
		bsn::model::Formula cost_expression;

		std::shared_ptr<web::http::client::http_client> client;

		bool connected;
};

#endif 