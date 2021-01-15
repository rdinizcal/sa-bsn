#ifndef G4T1_HPP
#define G4T1_HPP

#include <chrono>
#include <memory>
#include <map>

#include <ros/package.h>
#include "ros/ros.h"

#include "bsn/processor/Processor.hpp"
#include "bsn/utils/utils.hpp"

#include "component/CentralHub.hpp"

#include "archlib/target_system/Component.hpp"
#include "archlib/AdaptationCommand.h"

#include "messages/SensorData.h"
#include "messages/TargetSystemData.h"

class G4T1 : public CentralHub {
    
    public:
        G4T1(int &argc, char **argv, const std::string &name);
        virtual ~G4T1();

    private:
        G4T1(const G4T1 & /*obj*/);
        G4T1 &operator=(const G4T1 & /*obj*/);

        std::string makePacket();
        std::vector<std::string> getPatientStatus();
        int32_t getSensorId(std::string type);

    public:
        virtual void setUp();
        virtual void tearDown();   

        virtual void collect(const messages::SensorData::ConstPtr& sensor_data);
        virtual void process();
        virtual void transfer();

        void processDiagnostics(const messages::DiagnosticsData::ConstPtr& msg);

    private:
        double patient_status;

        double abps_risk;
        double abpd_risk;
        double oxi_risk;
        double ecg_risk;
        double trm_risk;

        double abps_batt;
        double abpd_batt;
        double oxi_batt;
        double ecg_batt;
        double trm_batt;

        double abps_raw;
        double abpd_raw;
        double oxi_raw;
        double ecg_raw;
        double trm_raw;

        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Publisher chDetectPub;

        bool lost_packt;

        int32_t currentDataId;
        std::string currentType;
        std::map<std::string, uint32_t> prevId;

};

#endif 