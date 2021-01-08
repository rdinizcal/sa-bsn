#include <DiagnosticsAnalyzer.hpp>

DiagnosticsAnalyzer::DiagnosticsAnalyzer(int  &argc, char **argv, std::string name) {
    ros::init(argc, argv, name, ros::init_options::NoSigintHandler);
}

DiagnosticsAnalyzer::~DiagnosticsAnalyzer() {}

void DiagnosticsAnalyzer::setUp() {

    ros::NodeHandle nh;
    sensorSub = nh.subscribe("sensor_diagnostics", 10, &DiagnosticsAnalyzer::processSensorData, this);
    centralhubSub = nh.subscribe("centralhub_diagnostics", 10, &DiagnosticsAnalyzer::processCentralhubData, this);
}

void DiagnosticsAnalyzer::processCentralhubData(const messages::DiagnosticsData::ConstPtr& msg) {}

void DiagnosticsAnalyzer::processSensorData(const messages::DiagnosticsData::ConstPtr& msg) {
    std::cout << "id: " + msg->id;
    std::cout << ",from: " + msg->sensor;
    std::cout << ", in state: " + msg->state;
    std::cout << ", data: " << msg->data << std::endl;
}

void DiagnosticsAnalyzer::tearDown() {}
void DiagnosticsAnalyzer::body() {
    std::cout << "reached body" << std::endl;
}


int32_t DiagnosticsAnalyzer::run() {
        setUp();

        ros::Rate loop_rate(10);
        
        while(ros::ok()) {
            ros::spinOnce();
            body();
            loop_rate.sleep();
        }

        tearDown();
        return 0;
}