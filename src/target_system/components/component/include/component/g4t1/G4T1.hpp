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

#include "component/CentralHub.hpp"

#include "archlib/target_system/Component.hpp"
#include "archlib/AdaptationCommand.h"

#include "messages/SensorData.h"
#include "messages/TargeSystemData.h"

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

        double bpr_risk;
        double oxi_risk;
        double ecg_risk;
        double trm_risk;

        double bpr_batt;
        double oxi_batt;
        double ecg_batt;
        double trm_batt;

        ros::Publisher pub;
};

#endif 