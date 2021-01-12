#include <PropertyAnalyzer.hpp>

PropertyAnalyzer::PropertyAnalyzer(int  &argc, char **argv, std::string name) {
    ros::init(argc, argv, name, ros::init_options::NoSigintHandler);
}

PropertyAnalyzer::~PropertyAnalyzer() {}

void PropertyAnalyzer::setUp() {

    ros::NodeHandle nh;
    sensorSub = nh.subscribe("sensor_diagnostics", 10, &PropertyAnalyzer::processSensorData, this);
    centralhubSub = nh.subscribe("centralhub_diagnostics", 10, &PropertyAnalyzer::processCentralhubData, this);
    sensorOnSub = nh.subscribe("collect_status", 10, &PropertyAnalyzer::processSensorOn, this);

    init = true;
    ON_reached = false;
    OFF_reached = false;
    SECOND_reached = false;
    THIRD_reached = false;
    wait_second = false;
    wait_third = false;
    gotMessage["sensor"] = gotMessage["centralhub"] = false;
    property_satisfied = true;

    nh.getParam("SensorName", currentSensor);
    nh.getParam("property", currentProperty);

    defineStateNames();
    defineStateTypes();

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

void PropertyAnalyzer::defineStateTypes() {
    
    stateTypes[0] = currentProperty == "p10" ? "centralhub" : "sensor";
    stateTypes[1] = "centralhub";
}

void PropertyAnalyzer::processCentralhubData(const messages::CentralhubDiagnostics::ConstPtr& msg) {     
    //std::cout << msg->id << std::endl;
    //std::cout << msg->type << std::endl;
    //std::cout << msg->source << std::endl;
    //std::cout << msg->status << std::endl;
    if (msg->type == "sensor" && msg->source == sensorAlias[currentSensor]) {
        gotMessage["centralhub"] = true;
        if (msg->status == "on") {
            //ON_reached = true;
        } else if (msg->status == centralhubSignal) {
            outgoingId = msg->id;
            THIRD_reached = true;
            property_satisfied = outgoingId == incomingId;
        } else if (msg->status == "off") {
            //OFF_reached = true;
        }
        //std::cout << "centralhub is " << msg->status << std::endl;
    } else if (msg->type == "centralhub") {
        gotMessage["centralhub"] = true;
        std::cout << "received meta message from centralhub" << std::endl;
    }
}

void PropertyAnalyzer::processSensorData(const messages::DiagnosticsData::ConstPtr& msg) {
    if (currentProperty != "p10") {
        if (msg->source == sensorAlias[currentSensor]) {
            gotMessage["sensor"] = true;
            if (msg->status == "on") {
                ON_reached = true;
            } else if (msg->status == sensorSignal) {
                SECOND_reached = true;
                incomingId = msg->id;
            } else if (msg->status == "off") {
                OFF_reached = true;
            }
        }
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

void PropertyAnalyzer::busyWait(const std::string& type) {
    ros::Rate loop_rate(2);
    //printStack();
    while (!gotMessage[type]) {ros::spinOnce();loop_rate.sleep();}
    printStack();
    gotMessage[type] = false;
}

std::string PropertyAnalyzer::yesOrNo(bool state) {
    return state == true? "yes":"no";
}

void PropertyAnalyzer::printStack() {
    std::cout << "==========================================" << std::endl;
    std::cout << "Current state: " << currentState << std::endl;
    std::cout << "Incoming: " << incomingId << " Outgoing: " << outgoingId << std::endl;
    std::cout << "Property satisfied? " << yesOrNo(property_satisfied) << std::endl;
    std::cout << "==========================================" << std::endl;
}

void PropertyAnalyzer::body() {

    while (init == true) {
        currentState = "Observer was initialized";
        busyWait(stateTypes[0]);

        if(ON_reached == true) {
            init = false;
            ON_reached = false;
            SECOND_reached = false;
            wait_second = true;
    
            while(wait_second == true) {
                currentState = "Waiting for data to be " + sensorSignal;
                busyWait(stateTypes[0]);

                if(SECOND_reached == true) {
                    SECOND_reached = false;
                    wait_second = false;
                    wait_third = true;

                    while(wait_third == true) {
                        currentState = "Waiting for data to be " + centralhubSignal;
                        busyWait(stateTypes[1]);
                        if(OFF_reached == true || !property_satisfied) {
                            currentState = "ERROR! Data collected, but not processed";
                            wait_third = false;
                            exit(0);
                        }

                        if(THIRD_reached == true) {
                            THIRD_reached = false;
                            wait_third = false;
                            wait_second = true;
                        }

                    }
                } 
                if(OFF_reached == true) {
                    OFF_reached = false;
                    wait_second = false;
                    init = true;
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