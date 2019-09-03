#ifndef G4T1_HPP
#define G4T1_HPP

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

#include "component/SchedulableComponent.hpp"

#include "messages/SensorData.h"

class G4T1 : public SchedulableComponent {
    
    private:
        G4T1(const G4T1 & /*obj*/);
        G4T1 &operator=(const G4T1 & /*obj*/);

        void receiveSensorData(const messages::SensorData::ConstPtr&);

        std::string makePacket();
        void persistData(std::vector<std::string>&);
        std::vector<std::string> getPatientStatus();

    public:
        G4T1(const int32_t &argc, char **argv);
        virtual ~G4T1();

        void setUp();
        void tearDown();   
        void body();

        void sendEvent(const std::string &/*type*/, const std::string &/*description*/);
		void sendStatus(const std::string &/*key*/, const double &/*value*/);

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
        std::string acc_risk;

        std::string bpr_batt;
        std::string oxi_batt;
        std::string ecg_batt;
        std::string trm_batt;
        std::string acc_batt;
};

#endif 