#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <array>
#include <fstream>

#include "messages/DiagnosticsData.h"
#include "messages/CentralhubDiagnostics.h"

#include "archlib/Status.h"

class PropertyAnalyzer {
    public:
        PropertyAnalyzer(int &argc, char **argv, std::string name);
        ~PropertyAnalyzer();

        void setUp();
        void tearDown();
        void body();

        int32_t run();

        void processCentralhubData(const messages::CentralhubDiagnostics::ConstPtr&);
        void processSensorData(const messages::DiagnosticsData::ConstPtr&);
        void processSensorOn(const archlib::Status::ConstPtr&);
        void processSensorInRange(const messages::DiagnosticsData::ConstPtr&);
        void processCentralhubDetection(const messages::CentralhubDiagnostics::ConstPtr&);
        void processCentralhubOn(const archlib::Status::ConstPtr&);
        void busyWait(const std::string&);
        void printStack();
        void defineStateNames();
        void defineStateTypes();
        std::string yesOrNo(bool);

        bool isPropertySatisfied();
        void flushData(const messages::CentralhubDiagnostics::ConstPtr& msg);
        void flushData(const messages::DiagnosticsData::ConstPtr& msg);

    private:
        //ros::NodeHandle nh;
        ros::Subscriber sensorSub;
        ros::Subscriber centralhubSub;
        ros::Subscriber detectionSub;
        ros::Subscriber sensorOnSub;
        ros::Subscriber chDetectedSub;
        ros::Subscriber sensorInRangeSub;

        std::map<std::string, std::string> sensorAlias;

        std::string currentState;
        std::string currentSensor;
        std::string currentProperty;
        std::string currentProcessed;

        std::string sensorSignal;
        std::string centralhubSignal;

        std::array<std::string, 4> stateNames;
        std::array<std::string, 2> stateTypes;

        bool ON_reached;
        bool OFF_reached;
        bool SECOND_reached;
        bool THIRD_reached;

        bool init;
        bool wait_second, wait_third;
        bool property_satisfied;
        bool violation_flag;

        std::map<std::string, bool> gotMessage;
        std::map<std::string, uint32_t> prevIdList, currentIdList;
        std::map<std::string, std::string> prevStatusList, currentStatusList;
        std::map<std::string, uint32_t> expectedMessage;

        uint32_t incomingId;
        uint32_t outgoingId;
        uint32_t prevId, currentId;
        uint32_t expectedId;

        std::fstream fp;
        std::string filepath;
};