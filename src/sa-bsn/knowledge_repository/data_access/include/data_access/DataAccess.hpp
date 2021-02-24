#ifndef DATA_ACCESS_HPP
#define DATA_ACCESS_HPP

#include <fstream>
#include <chrono>
#include <deque>

#include "ros/ros.h"
#include <ros/package.h>

#include "libbsn/goalmodel/Node.hpp"
#include "libbsn/goalmodel/Goal.hpp"
#include "libbsn/goalmodel/Task.hpp"
#include "libbsn/goalmodel/Property.hpp"
#include "libbsn/goalmodel/LeafTask.hpp"
#include "libbsn/goalmodel/Context.hpp"
#include "libbsn/goalmodel/GoalTree.hpp"
#include "libbsn/model/Formula.hpp"
#include "libbsn/utils/utils.hpp"

#include "archlib/Persist.h"
#include "archlib/DataAccessRequest.h"
#include "archlib/ROSComponent.hpp"

#include "StatusMessage.hpp"
#include "EnergyStatusMessage.hpp"
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
		std::chrono::high_resolution_clock::time_point nowInSeconds() const;

		void persistEvent(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content);
		void persistStatus(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content);
		void persistEnergyStatus(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content);
		void persistUncertainty(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content);
		void persistAdaptation(const int64_t &timestamp, const std::string &source, const std::string &target, const std::string &content);

		void flush();

		//double calculateCost();
		double calculateReliability();

		std::string calculateComponentReliability(const std::string& component);
		std::string calculateComponentCost(const std::string& component, std::string req_name);
		void resetStatus();
		//void updateBatteries();
		//void updateCosts();
		void applyTimeWindow();
	public:
		virtual void setUp();
		virtual void tearDown();
		virtual void body();

		void receivePersistMessage(const archlib::Persist::ConstPtr& msg);
		bool processQuery(archlib::DataAccessRequest::Request &req, archlib::DataAccessRequest::Response &res);
		void processTargetSystemData(const messages::TargetSystemData::ConstPtr& msg);

	protected:
		ros::NodeHandle handle;
	
	private:
		ros::Subscriber handle_persist;
		ros::ServiceServer server;
		ros::Subscriber targetSystemSub;

		std::fstream fp;
		std::string event_filepath;
		std::string status_filepath;
		std::string energy_status_filepath;
		std::string uncertainty_filepath;
		std::string adaptation_filepath;

		int64_t logical_clock;

		std::vector<StatusMessage> statusVec;
		std::vector<EnergyStatusMessage> energystatusVec;
		std::vector<EventMessage> eventVec;
		std::vector<UncertaintyMessage> uncertainVec;
		std::vector<AdaptationMessage> adaptVec;

		std::map<std::string, std::deque<std::pair<std::chrono::high_resolution_clock::time_point, std::string>>> status;
		std::map<std::string, std::deque<std::string>> events;
		int buffer_size;

		std::map<std::string, double> components_reliabilities;
		std::map<std::string, double> components_batteries;
		std::map<std::string, double> components_costs_engine, components_costs_enactor;
		std::map<std::string, uint32_t> contexts;

		std::string reliability_formula;
		std::string cost_formula;

		double frequency;
		int32_t count_to_calc_and_reset;
		int32_t count_to_fetch;
		int32_t arrived_status;
};

#endif 