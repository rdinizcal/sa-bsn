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
#include <ros/callback_queue.h>

#include "bsn/processor/Processor.hpp"
#include "bsn/operation/Operation.hpp"

#include "component/CentralHub.hpp"

#include "archlib/target_system/Component.hpp"
#include "archlib/AdaptationCommand.h"

#include "messages/SensorData.h"

class G4T1 : public CentralHub {
    
    public:
        G4T1(int &argc, char **argv, const std::string &name);
        virtual ~G4T1();

    private:
        G4T1(const G4T1 & /*obj*/);
        G4T1 &operator=(const G4T1 & /*obj*/);

        std::string makePacket();
        std::vector<std::string> getPatientStatus();

    public:
        virtual void setUp();
        virtual void tearDown();   

        virtual void collect(const messages::SensorData::ConstPtr& sensor_data);
        virtual void process();
        virtual void transfer();

    private:
        bool connect;
        std::string database_url;
        
        bool lost_packt;
        double patient_status;
        int session;

        std::string bpr_risk;
        std::string oxi_risk;
        std::string ecg_risk;
        std::string trm_risk;

        std::string bpr_batt;
        std::string oxi_batt;
        std::string ecg_batt;
        std::string trm_batt;
};

#endif 