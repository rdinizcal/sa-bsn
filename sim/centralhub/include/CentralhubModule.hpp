#ifndef CENTRALHUB_MODULE_HPP
#define CENTRALHUB_MODULE_HPP

#include <fstream>
#include <chrono>
#include <memory>
#include <map>

#include <cpprest/http_client.h>
#include <cpprest/json.h>
#include <ros/package.h>
#include "ros/ros.h"

#include "bsn/processor/Processor.hpp"
#include "bsn/operation/Operation.hpp"

#include "messages/SensorData.h"
#include "messages/SystemInfo.h"


class CentralhubModule {
    private:
        CentralhubModule(const CentralhubModule & /*obj*/);
        CentralhubModule &operator=(const CentralhubModule & /*obj*/);
        virtual void tearDown();   

		void sendTaskInfo(const std::string &/*task_id*/, const double &/*cost*/, const double &/*reliability*/, const double &/*frequency*/);
		
        void sendMonitorTaskInfo(const std::string &/*task_id*/, const double &/*cost*/, const double &/*reliability*/, const double &/*frequency*/);

        void receiveSensorData(const messages::SensorData::ConstPtr&);
        void receiveSystemInfo(const messages::SystemInfo::ConstPtr&); 

        std::string makePacket();
        void persistData(std::vector<std::string>&);
        std::vector<std::string> getPatientStatus();

    public:
        void setUp();
        CentralhubModule(const int32_t &argc, char **argv);
        virtual ~CentralhubModule();

        void run();

    private:
        bool active;
		std::map<std::string,double> params;

        bool connect;
        std::string database_url;
        
        bool persist;
        std::ofstream fp;
        std::string path;

        std::array<double, 5> data;
        std::vector<std::list<double>> data_list;
        double patient_status;
        int session;

        std::string bpr_risk;
        std::string oxi_risk;
        std::string ecg_risk;
        std::string trm_risk;
        std::string reliability;
        std::string cost;

};

#endif 