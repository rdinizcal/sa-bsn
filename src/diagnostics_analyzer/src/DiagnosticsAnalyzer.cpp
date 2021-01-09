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

void DiagnosticsAnalyzer::processCentralhubData(const messages::DiagnosticsData::ConstPtr& msg) {
    //std::cout << "id: " + std::to_string(msg->id);
    //std::cout << ",from: " + msg->sensor;
    //std::cout << ", in state: " + msg->state;
    //std::cout << ", data: " << msg->data << std::endl;

    if (msg->id != currentCollectedId[msg->sensor]) {
        std::cout << "p7: "<<msg->sensor <<": " << currentCollectedId[msg->sensor] << ", got: " << msg->id;
        std::cout << "data: " << msg->data << std::endl;
        //std::cout << "Property P7 not OK! id: " << std::to_string(msg->id) << std::endl;
    } if (msg->id != currentSentId[msg->sensor]) {
        std::cout << "p9: "<< msg->sensor <<": " << currentSentId[msg->sensor] << ", got: " << msg->id;
        std::cout << " data: " << msg->data << std::endl;
        //std::cout << "Property P9 not OK! id: " << std::to_string(msg->id) << std::endl;
    }
}

void DiagnosticsAnalyzer::processSensorData(const messages::DiagnosticsData::ConstPtr& msg) {
    //std::cout << "id: " + std::to_string(msg->id);
    //std::cout << ",from: " + msg->sensor;
    //std::cout << ", in state: " + msg->state;
    //std::cout << ", data: " << msg->data << std::endl;

    if (msg->state == "collect") {
        currentCollectedId[msg->sensor] = msg->id;
        //expectedCollectedId[msg->sensor]++;
    } else if (msg->state == "sent") {
        currentSentId[msg->sensor] = msg->id;
        //expectedSentId[msg->sensor]++;
    }
}

void DiagnosticsAnalyzer::tearDown() {}
void DiagnosticsAnalyzer::body() {}


int32_t DiagnosticsAnalyzer::run() {
        setUp();
        
        ros::Rate loop_rate(5);
        
        while(ros::ok()) {
            ros::spinOnce();
            body();
            loop_rate.sleep();
        }

        tearDown();
        return 0;
}