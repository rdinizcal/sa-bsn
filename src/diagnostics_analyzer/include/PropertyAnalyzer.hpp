#include <ros/ros.h>
#include <ros/console.h>

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include "messages/DiagnosticsData.h"
#include "messages/DiagnosticsStatus.h"
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

        void processCentralhubData(const messages::DiagnosticsData::ConstPtr&);
        void processSensorData(const messages::DiagnosticsData::ConstPtr&);
        void processSensorStatus(const messages::DiagnosticsStatus::ConstPtr&);
        void processSensorOn(const archlib::Status::ConstPtr&);
        void printStack();
        std::string yesOrNo(bool);
        void busyWait();

        //ros::NodeHandle nh;
        ros::Subscriber sensorSub;
        ros::Subscriber centralhubSub;
        ros::Subscriber sensorStatusSub;
        ros::Subscriber sensorOnSub;

        std::map<std::string, std::string> sensorAlias;

        std::string currentState;
        std::string currentSensor;

        bool init;
        bool ON_reached;
        bool OFF_reached;
        bool COLLECTED_reached;
        bool wait_collect, wait_process;
        bool PROCESSED_reached;
        bool property_satisfied;

        bool gotMessage;
};