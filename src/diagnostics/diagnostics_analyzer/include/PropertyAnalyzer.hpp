#include <ros/ros.h>
#include <ros/console.h>

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <array>

#include "messages/DiagnosticsData.h"
#include "messages/CentralhubDiagnostics.h"
#include "messages/LogMessage.h"
#include "archlib/Status.h"

class PropertyAnalyzer {
    public:
        PropertyAnalyzer(int &argc, char **argv, std::string name);
        ~PropertyAnalyzer();

        void setUp();
        void tearDown();
        void body();

        int32_t run();

    private:

        void processCentralhubData(const messages::CentralhubDiagnostics::ConstPtr&);
        void processSensorData(const messages::DiagnosticsData::ConstPtr&);
        void processSensorOn(const archlib::Status::ConstPtr&);
        void processCentralhubDetection(const messages::CentralhubDiagnostics::ConstPtr&);
        void processCentralhubOn(const archlib::Status::ConstPtr&);
        void busyWait(const std::string&);
        void printStack();
        void defineStateNames();
        void defineStateTypes();
        std::string yesOrNo(bool);

        bool isPropertySatisfied();

        //ros::NodeHandle nh;
        ros::Subscriber sensorSub;
        ros::Subscriber centralhubSub;
        ros::Subscriber detectionSub;
        ros::Subscriber sensorOnSub;
        ros::Subscriber chDetectedSub;

        ros::Publisher logPub;

        std::map<std::string, std::string> sensorAlias;

        std::string currentState;
        std::string currentSensor;
        std::string currentProperty;

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

        std::map<std::string, uint32_t> expectedMessage;

        std::map<std::string, bool> gotMessage;
        uint32_t incomingId;
        uint32_t outgoingId;

        std::string msgSource;
        std::string msgStatus;
        ros::Time msgTimestamp;
};