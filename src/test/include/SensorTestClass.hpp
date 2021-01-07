#ifndef SENSOR_TEST_HPP
#define SENSOR_TEST_HPP

#include "messages/SensorFrequency.h"
#include "archlib/Status.h"
#include "archlib/AdaptationCommand.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>

class SensorTestClass {

    std::string name;
    std::string alias;
    double freqVal;
    double data;
    bool receivedMessage;

    public:
        SensorTestClass();
        SensorTestClass(std::string name, std::string alias, double freqVal);
        virtual ~SensorTestClass();

        double getFreq();
        void setFreq(double);

        std::string getName();
        std::string getAlias();

        bool getReceivedMessage();
        void setReceivedMessage(bool);

        //bool getPatientData(services::PatientData::Request &request, services::PatientData::Response &response);
        void freqCallback(const messages::SensorFrequency::ConstPtr&);
        void frequencyTestSetup(std::string val, std::shared_ptr<ros::NodeHandle>);
};

#endif 