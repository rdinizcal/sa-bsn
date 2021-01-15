#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <array>

#include "messages/LogMessage.h"
#include "archlib/Status.h"

#include <chrono>

class DiagnosticsLogger {
    public:
        DiagnosticsLogger(int &argc, char **argv, std::string name);
        ~DiagnosticsLogger();

        void setUp();
        void tearDown();
        void body();

        int32_t run();

    private:

        void processLog(const messages::LogMessage& );
        int64_t now() const;
        

        std::string filepath;
        std::string currentProperty;
        std::fstream fp;


        ros::Subscriber loggerSub;
};