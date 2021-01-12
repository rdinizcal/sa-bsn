#include <PropertyAnalyzer.hpp>

PropertyAnalyzer::PropertyAnalyzer(int  &argc, char **argv, std::string name) {
    ros::init(argc, argv, name, ros::init_options::NoSigintHandler);
}

PropertyAnalyzer::~PropertyAnalyzer() {}

void PropertyAnalyzer::setUp() {

    ros::NodeHandle nh;
    sensorSub = nh.subscribe("sensor_diagnostics", 10, &PropertyAnalyzer::processSensorData, this);
    centralhubSub = nh.subscribe("centralhub_diagnostics", 10, &PropertyAnalyzer::processCentralhubData, this);
    //sensorStatusSub = nh.subscribe("sensor_status", 10, &PropertyAnalyzer::processSensorStatus, this);
    sensorOnSub = nh.subscribe("collect_status", 10, &PropertyAnalyzer::processSensorOn, this);

    init = true;
    ON_reached = false;
    OFF_reached = false;
    COLLECTED_reached = false;
    PROCESSED_reached = false;
    wait_collect = false;
    wait_process = false;
    gotMessage = false;
    property_satisfied = true;

    nh.getParam("SensorName", currentSensor);
    nh.getParam("property", currentProperty);

    defineStateNames();

    sensorAlias["/g3t1_1"] = "oximeter";
    sensorAlias["/g3t1_2"] = "ecg";
    sensorAlias["/g3t1_3"] = "thermometer";
    sensorAlias["/g3t1_4"] = "abps";
    sensorAlias["/g3t1_5"] = "abpd";
 
    std::cout << "Monitoring property: " << currentProperty;
    std::cout << " on sensor: " << sensorAlias[currentSensor] << std::endl;
}

void PropertyAnalyzer::defineStateNames() {
    
    stateNames[0] = "init";
    stateNames[3] = "ERROR";

    if (currentProperty == "p7") {
        stateNames[1] = "wait_collect";
        stateNames[2] = "wait_process";
        sensorSignal = "collected";
        centralhubSignal = "processed";
    } else if (currentProperty == "p8") {
        stateNames[1] = "wait_collected";
        stateNames[2] = "wait_persisted";
        sensorSignal = "collected";
        centralhubSignal = "persisted";
    } else if (currentProperty == "p9") {
        stateNames[1] = "wait_sent";
        stateNames[2] = "wait_process";
        sensorSignal = "sent";
        centralhubSignal = "processed";
    } else if (currentProperty == "p10") {
        stateNames[1] = "wait_process";
        stateNames[2] = "wait_detected";
        sensorSignal = "processed";
        centralhubSignal = "detected";
    }
}

void PropertyAnalyzer::processCentralhubData(const messages::DiagnosticsData::ConstPtr& msg) {     
    if (msg->source == "centralhub") {
        if (msg->status == "on") {
            ON_reached = true;
        } else if (msg->status == centralhubSignal) {
            outgoingId = msg->id;
            PROCESSED_reached = true;
            gotMessage = true;
        } else if (msg->status == "off") {
            OFF_reached = true;
        }
    }
}

void PropertyAnalyzer::processSensorData(const messages::DiagnosticsData::ConstPtr& msg) {
    if (msg->source == sensorAlias[currentSensor]) {
        if (msg->status == "on") {
            ON_reached = true;
        } else if (msg->status == sensorSignal) {
            COLLECTED_reached = true;
            incomingId = msg->id;
        } else if (msg->status == "off") {
            OFF_reached = true;
        }
        gotMessage = true;
    }
}

void PropertyAnalyzer::processSensorOn(const archlib::Status::ConstPtr& msg) {
    
    if (msg->source == currentSensor) {
        if (msg->content == "init" && ON_reached == false) {
            ON_reached = true;
            printStack();
        }
    }
}

void PropertyAnalyzer::tearDown() {}

void PropertyAnalyzer::busyWait() {
    ros::Rate loop_rate(2);
    printStack();
    while (!gotMessage) {ros::spinOnce();loop_rate.sleep();}
    printStack();
    gotMessage = false;
}

std::string PropertyAnalyzer::yesOrNo(bool state) {
    return state == true? "yes":"no";
}

void PropertyAnalyzer::printStack() {
    std::cout << "==========================================" << std::endl;
    std::cout << "Current state: " << currentState << std::endl;
    std::cout << "Incoming: " << incomingId << " Outgoing: " << outgoingId << std::endl;
    std::cout << "ON_reached: " << yesOrNo(ON_reached) << std::endl;
    std::cout << "COLLECTED_reached: " << yesOrNo(COLLECTED_reached) << std::endl;
    std::cout << "PROCESSED_reached: " << yesOrNo(PROCESSED_reached) << std::endl;
    std::cout << "OFF_reached: " << yesOrNo(OFF_reached) << std::endl;
    std::cout << "Property satisfied? " << yesOrNo(property_satisfied) <<std::endl;
    std::cout << "==========================================" << std::endl;
}

void PropertyAnalyzer::body() {

    while (init == true) {
        currentState = "Observer was initialized";
        busyWait();

        if(ON_reached == true) {
            init = false;
            ON_reached = false;
            wait_collect = true;
    
            while(wait_collect == true) {
                currentState = "Waiting for data to be collected";
                busyWait();

                if(COLLECTED_reached == true) {
                    COLLECTED_reached = false;
                    wait_collect = false;
                    wait_process = true;

                    while(wait_process == true) {
                        currentState = "Waiting for data to be processed";
                        busyWait();
                        if(PROCESSED_reached == true) {
                            PROCESSED_reached = false;
                            wait_process = false;
                            wait_collect = true;
                        }

                        if(OFF_reached == true) {
                            currentState = "ERROR! Data collected, but not processed";
                            property_satisfied = false;
                            wait_process = false;
                            printStack();
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

int32_t PropertyAnalyzer::run() {
        setUp();
        
        ros::Rate loop_rate(2);
        
        while(ros::ok()) {
            body();
            ros::spinOnce();
            loop_rate.sleep();
        }

        tearDown();
        return 0;
}