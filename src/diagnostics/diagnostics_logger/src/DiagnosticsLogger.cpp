#include <DiagnosticsLogger.hpp>

DiagnosticsLogger::DiagnosticsLogger(int  &argc, char **argv, std::string name) {
    ros::init(argc, argv, name);
}

DiagnosticsLogger::~DiagnosticsLogger() {}

void DiagnosticsLogger::setUp() {

    ros::NodeHandle nh;
    std::string path = ros::package::getPath("diagnostics_logger");

    loggerSub = nh.subscribe("diagnostics_log", 1000, &DiagnosticsLogger::processLog,this); 
    
    nh.getParam("property", currentProperty);
    filepath = path + "/../logs/" + currentProperty + "_" + std::to_string(now()) + ".log";


    fp.open(filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "\n";
    fp.close();
}

int64_t DiagnosticsLogger::now() const {
    return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

void DiagnosticsLogger::tearDown() {}

void DiagnosticsLogger::processLog(const messages::LogMessage& msg) {

}

void DiagnosticsLogger::body() {
}

int32_t DiagnosticsLogger::run() {
        setUp();
        
        ros::Rate loop_rate(1.5);
        
        while(ros::ok()) {
            body();
            ros::spinOnce();
            loop_rate.sleep();
        }

        tearDown();
        return 0;
}