#ifndef SENSOR_TEST_HPP
#define SENSOR_TEST_HPP

#include "messages/SensorFrequency.h"
#include "archlib/Status.h"

#include <ros/ros.h>
#include <ros/package.h>


class SensorTest {

    std::string name;
    std::string alias;
    double freqVal;
    double data;
    bool receivedMessage;

    public:
		SensorTest(std::string name, std::string alias, double freqVal);
        virtual ~SensorTest();

        double getFreq();
        void setFreq(double);

        std::string getName();
        std::string getAlias();

        bool getReceivedMessage();
        void setReceivedMessage(bool);

        void freqCallback(const messages::SensorFrequency::ConstPtr&);
        void frequencyTestSetup(std::string val);
};

#endif 