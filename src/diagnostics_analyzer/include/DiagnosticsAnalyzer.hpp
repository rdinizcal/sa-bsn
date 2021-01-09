#include <ros/ros.h>
#include <ros/console.h>

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include "messages/DiagnosticsData.h"

class DiagnosticsAnalyzer {
    public:
        DiagnosticsAnalyzer(int &argc, char **argv, std::string name);
        ~DiagnosticsAnalyzer();

        void setUp();
        void tearDown();
        void body();

        int32_t run();

    private:

        void processCentralhubData(const messages::DiagnosticsData::ConstPtr&);
        void processSensorData(const messages::DiagnosticsData::ConstPtr&);

        //ros::NodeHandle nh;
        ros::Subscriber sensorSub;
        ros::Subscriber centralhubSub;
        
        //int32_t collectedId;
        //int32_t processedId;
        std::map<std::string, int> currentCollectedId;
        std::map<std::string, int> currentSentId;

        std::map<std::string, int> expectedCollectedId;
        std::map<std::string, int> expectedSentId;

};