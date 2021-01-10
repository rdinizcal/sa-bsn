#include <DiagnosticsAnalyzer.hpp>

DiagnosticsAnalyzer::DiagnosticsAnalyzer(int  &argc, char **argv, std::string name) {
    ros::init(argc, argv, name, ros::init_options::NoSigintHandler);
}

DiagnosticsAnalyzer::~DiagnosticsAnalyzer() {}

void DiagnosticsAnalyzer::setUp() {

    ros::NodeHandle nh;
    sensorSub = nh.subscribe("sensor_diagnostics", 10, &DiagnosticsAnalyzer::processSensorData, this);
    centralhubSub = nh.subscribe("centralhub_diagnostics", 10, &DiagnosticsAnalyzer::processCentralhubData, this);
    sensorStatusSub = nh.subscribe("sensor_status", 10, &DiagnosticsAnalyzer::processSensorStatus, this);
    sensorOnSub = nh.subscribe("collect_status", 10, &DiagnosticsAnalyzer::processSensorOn, this);

    init = true;
    ON_reached = false;
    OFF_reached = false;
    COLLECTED_reached = false;
    PROCESSED_reached = false;
    wait_collect = false;
    wait_process = false;
    gotMessage = false;
}

void DiagnosticsAnalyzer::processCentralhubData(const messages::DiagnosticsData::ConstPtr& msg) {
    //std::cout << "id: " + std::to_string(msg->id);
    //std::cout << ",from: " + msg->sensor;
    //std::cout << ", in state: " + msg->state;
    //std::cout << ", data: " << msg->data << std::endl;

    //if (msg->id != currentCollectedId[msg->sensor]) {
    //    std::cout << "p7: "<<msg->sensor <<": " << currentCollectedId[msg->sensor] << ", got: " << msg->id;
    //    std::cout << "data: " << msg->data << std::endl;
    //    //std::cout << "Property P7 not OK! id: " << std::to_string(msg->id) << std::endl;
    //} if (msg->id != currentSentId[msg->sensor]) {
    //    std::cout << "p9: "<< msg->sensor <<": " << currentSentId[msg->sensor] << ", got: " << msg->id;
    //    std::cout << " data: " << msg->data << std::endl;
    //    //std::cout << "Property P9 not OK! id: " << std::to_string(msg->id) << std::endl;
    //}
}

void DiagnosticsAnalyzer::processSensorData(const messages::DiagnosticsData::ConstPtr& msg) {
    //std::cout << "id: " + std::to_string(msg->id);
    //std::cout << ",from: " + msg->sensor;
    //std::cout << ", in state: " + msg->state;
    //std::cout << ", data: " << msg->data << std::endl;

    //if (msg->state == "collect") {
    //    currentCollectedId[msg->sensor] = msg->id;
    //    //expectedCollectedId[msg->sensor]++;
    //} else if (msg->state == "sent") {
    //    currentSentId[msg->sensor] = msg->id;
    //    //expectedSentId[msg->sensor]++;
    //}
}

void DiagnosticsAnalyzer::processSensorStatus(const messages::DiagnosticsStatus::ConstPtr& msg) {
    //std::cout << msg->sensor << " is " << msg->status << std::endl;

    gotMessage = true;
    if (msg->sensor == "centralhub") {
        if (msg->status == "processed") {
            PROCESSED_reached = true;
        }
    } else {
        if (msg->status == "on") {
            ON_reached = true;
        } else if (msg->status == "collected") {
            COLLECTED_reached = true;
        } else if (msg->status == "off") {
            OFF_reached = true;
        }
    }
}

void DiagnosticsAnalyzer::processSensorOn(const archlib::Status::ConstPtr& msg) {
    
    //if (msg->source == "/g3t1_1") {
    //    if (msg->content == "init" && ON_reached == false) {
    //        ON_reached = true;
    //    }
    //}
}

void DiagnosticsAnalyzer::tearDown() {}

void DiagnosticsAnalyzer::busyWait() {
    ros::Rate loop_rate(5);
    while (!gotMessage) {ros::spinOnce();loop_rate.sleep();}
    gotMessage = false;
}

void DiagnosticsAnalyzer::body() {

    while (init == true) {
        std::cout << "on state: init" << std::endl;
        busyWait();

        if(ON_reached == true) {
            init = false;
            ON_reached = false;
            wait_collect = true;
    
            while(wait_collect == true) {
                std::cout << "on state: wait_collect" << std::endl;
                busyWait();

                if(COLLECTED_reached == true) {
                    COLLECTED_reached = false;
                    wait_collect = false;
                    wait_process = true;

                    while(wait_process == true) {
                        std::cout << "on state: wait_process" << std::endl;

                        busyWait();
                        if(PROCESSED_reached == true) {
                            PROCESSED_reached = false;
                            wait_process = false;
                            wait_collect = true;
                        }

                        if(OFF_reached == true) {
                            std::cout << "on state: ERROR!" << std::endl;
                            wait_process = false;
                        }
                    }
                } 
                if(OFF_reached == true) {
                    init = true;
                    OFF_reached = false;
                    wait_collect = false;
                } 
            }
        }
    }
}

int32_t DiagnosticsAnalyzer::run() {
        setUp();
        
        ros::Rate loop_rate(5);
        
        while(ros::ok()) {
            body();
            ros::spinOnce();
            loop_rate.sleep();
        }

        tearDown();
        return 0;
}