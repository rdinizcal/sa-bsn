#include <PropertyAnalyzer.hpp>

PropertyAnalyzer::PropertyAnalyzer(int  &argc, char **argv, std::string name) {
    ros::init(argc, argv, name, ros::init_options::NoSigintHandler);
}

PropertyAnalyzer::~PropertyAnalyzer() {}

void PropertyAnalyzer::setUp() {

    ros::NodeHandle nh;
    sensorSub = nh.subscribe("sensor_diagnostics", 100, &PropertyAnalyzer::processSensorData, this);
    centralhubSub = nh.subscribe("centralhub_diagnostics", 100, &PropertyAnalyzer::processCentralhubData, this);
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
    
    if (currentProperty == "p10") chDetectedSub = nh.subscribe("ch_detected", 10, &PropertyAnalyzer::processCentralhubDetection, this);

    std::string path = ros::package::getPath("diagnostics_analyzer");

    filepath = path + "/../logs/"+currentProperty+"/observer/observer_"+sensorAlias[currentSensor]+".log";
    std::cout << filepath << std::endl;

    fp.open(filepath, std::fstream::in | std::fstream::out | std::fstream::trunc);
    fp << "\n";
    fp.close();
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
    
    stateTypes[0] = currentProperty == "p10" ? "centralhub_processed" : "sensor";
    stateTypes[1] = "centralhub";
}

void PropertyAnalyzer::processCentralhubDetection(const messages::CentralhubDiagnostics::ConstPtr& msg) {
    if (currentProperty == "p10") {
        if (msg->type == "centralhub") {
            if (msg->status == "on" || msg->status == "processed"
             || msg->status == "detected" || msg->status == "off") {
                gotMessage["centralhub"] = true;

                if (msg->status == "processed") {
                    incomingId = msg->id;
                    SECOND_reached = true;
                    gotMessage["centralhub_processed"] = true;
                } else if (msg->status == "detected") {
                    outgoingId = msg->id;
                    THIRD_reached = true;
                    property_satisfied = outgoingId == incomingId;
                    gotMessage["centralhub"] = true;
                } else if (msg->status == "on") {
                    ON_reached = true;
                } else if (msg->status == "off") {
                    OFF_reached = true;
                }
                fp.open(filepath, std::fstream::in | std::fstream::out | std::fstream::app);
                fp << msg->timestamp << ",";
                fp << msg->id << ",";
                fp << msg->source << ",";
                fp << msg->status << std::endl;
                fp.close();
                
            }
        }
    }
}

void PropertyAnalyzer::processCentralhubData(const messages::CentralhubDiagnostics::ConstPtr& msg) {     
    if (currentProperty != "p10") {
        if (msg->type == "sensor" && msg->source == sensorAlias[currentSensor]) {
            if (msg->status == "on" || msg->status == centralhubSignal || msg->status == "off") {
                gotMessage["centralhub"] = true;
                
                if (msg->status == centralhubSignal) {
                    outgoingId = msg->id;
                    THIRD_reached = true;
                    if (property_satisfied) property_satisfied = outgoingId == incomingId;
                }
                if (currentId != prevId) {
                    fp.open(filepath, std::fstream::in | std::fstream::out | std::fstream::app);
                    fp << msg->timestamp << ",";
                    fp << msg->id << ",";
                    fp << "centralhub" << ",";
                    fp << msg->status << ",";
                    fp << property_satisfied << std::endl; 
                    fp.close();
                }
                    prevId = currentId;
            }
        } else if (msg->type == "centralhub") {
            gotMessage["centralhub"] = true;
            std::cout << "received meta message from centralhub" << std::endl;
        }
    }
}

void PropertyAnalyzer::processSensorData(const messages::DiagnosticsData::ConstPtr& msg) {
    if (currentProperty != "p10") {
        if (msg->source == sensorAlias[currentSensor]) {
            if (msg->status == "on" || msg->status == sensorSignal || msg->status == "off") {
                gotMessage["sensor"] = true;
                if (msg->status == "on") {
                    ON_reached = true;
                } else if (msg->status == sensorSignal) {
                    SECOND_reached = true;
                    incomingId = msg->id;
                } else if (msg->status == "off") {
                    OFF_reached = true;
                }
                currentId = msg->id;
                fp.open(filepath, std::fstream::in | std::fstream::out | std::fstream::app);
                fp << msg->timestamp << ",";
                fp << msg->id << ",";
                fp << msg->source << ",";
                fp << msg->status << ",";
                fp << property_satisfied << std::endl;
                fp.close();
            }
        }
    }
}

void PropertyAnalyzer::processSensorOn(const archlib::Status::ConstPtr& msg) {
    if (currentProperty == "p10") {    
        if (msg->source == "/g4t1") {
            if (msg->content == "init" && ON_reached == false) {
                ON_reached = true;
                printStack();

                fp.open(filepath, std::fstream::in | std::fstream::out | std::fstream::app);
                fp << msg->timestamp << ",";
                fp << 0 << ",";
                fp << "centralhub" << ",";
                fp << "on" << ",";
                fp << property_satisfied << std::endl;
                fp.close();    
            }
        }
    } else if (currentProperty != "p10") {    
        if (msg->source == currentSensor) {
            if (msg->content == "init" && ON_reached == false) {            
                ON_reached = true;
                printStack();

                fp.open(filepath, std::fstream::in | std::fstream::out | std::fstream::app);
                fp << msg->timestamp << ",";
                fp << 0 << ",";
                fp << sensorAlias[currentSensor] << ",";
                fp << "on" << ",";
                fp << property_satisfied << std::endl;
                fp.close();
            }
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
                            currentState = "ERROR! Data "+sensorSignal+", but not "+centralhubSignal;
                            wait_third = false;
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