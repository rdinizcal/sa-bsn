#ifndef CENTRALHUB_TEST_HPP
#define CENTRALHUB_TEST_HPP

#include "messages/TargetSystemData.h"
#include "messages/SensorData.h"
#include "archlib/AdaptationCommand.h"

#include <ros/ros.h>
#include <ros/package.h>


class CentralhubTestClass {

    double patientRisk;
    double trmBatt;
    double ecgBatt;
    double oxiBatt;
    double abpsBatt;
    double abpdBatt;
    double trmRisk;
    double ecgRisk;
    double oxiRisk;
    double abpsRisk;
    double abpdRisk;
    double trmData;
    double ecgData;
    double oxiData;
    double abpsData;
    double abpdData;
    double patientStatus;

    public:
        CentralhubTestClass();
        virtual ~CentralhubTestClass();

        void setupCentralhubTest(messages::SensorData thermMsg,
            messages::SensorData oxiMsg,
            messages::SensorData ecgMsg,
            messages::SensorData abpsMsg,
            messages::SensorData abpdMsg,
            std::shared_ptr<ros::NodeHandle> nh
        );

        void setTherm(messages::SensorData);
        void setEcg(messages::SensorData);
        void setOxi(messages::SensorData);
        void setAbps(messages::SensorData);
        void setAbpd(messages::SensorData);

        void processTargetSystemData(const messages::TargetSystemData::ConstPtr &msg);
        double getPatientRisk();
        void setPatientRisk(double patientRisk);
        double getTrmBatt();
        void setTrmBatt(double trmBatt);
        double getEcgBatt();
        void setEcgBatt(double ecgBatt);
        double getOxiBatt(); 
        void setOxiBatt(double oxiBatt);
        double getAbpsBatt();
        void setAbpsBatt(double abpsBatt);
        double getAbpdBatt();
        void setAbpdBatt(double abpdBatt);
        double getTrmRisk();
        void setTrmRisk(double trmRisk);
        double getEcgRisk();
        void setEcgRisk(double ecgRisk);
        double getOxiRisk();
        void setOxiRisk(double oxiRisk);
        double getAbpsRisk();
        void setAbpsRisk(double abpsRisk);
        double getAbpdRisk();
        void setAbpdRisk(double abpdRisk);
        double getTrmData();
        void setTrmData(double trmData);
        double getEcgData();
        void setEcgData(double ecgData);
        double getOxiData();
        void setOxiData(double oxiData);
        double getAbpsData();
        void setAbpsData(double abpsData);
        double getAbpdData();
        void setAbpdData(double abpdData);
        double getPatientStatus();
        void setPatientStatus(double patientStatus);
};

#endif 