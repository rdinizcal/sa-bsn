#include "component/g4t1/G4T1.hpp"

using namespace bsn::processor;

G4T1::G4T1(int &argc, char **argv, const std::string &name) :
    Component(argc, argv, name),
    active(true),
    params({{"freq",1}}),
    connect(true),
    database_url(),
    persist(true),
    fp(),
    path("centralhub_output.csv"),
    data(),
    data_list(5),
    patient_status(0.0) {}
	
G4T1::~G4T1() {}

void G4T1::setUp() {
    ros::NodeHandle configHandler;
    configHandler.getParam("connect", connect);
    configHandler.getParam("db_url", database_url);
    configHandler.getParam("persist", persist);
    configHandler.getParam("path", path);
    configHandler.getParam("session", session);

    if (persist) {
        fp.open(path);
        fp << "ID,SPO2_DATA,ECG_DATA,TEMP_DATA,BLOODPRESSURE_DATA,OVERALL_STATUS,PATIENT_STATE,TIME_MS" << std::endl;
    }
    for(std::vector<std::list<double>>::iterator it = data_list.begin();
        it != data_list.end(); ++it) {
            (*it).push_back(0.0);
    }

    { // Configure module descriptor for scheduling
        double freq;
        rosComponentDescriptor.setName(ros::this_node::getName());

        configHandler.getParam("frequency", freq);
        rosComponentDescriptor.setFreq(freq);
    }
}

void G4T1::tearDown() {
    if (persist)
        fp.close();
}

std::string G4T1::makePacket() {
    //std::string packet = "88,97#10,20,30,40,50&";
    std::string packet = "";

    packet.append(trm_batt).append(",");
    packet.append(ecg_batt).append(",");
    packet.append(oxi_batt).append(",");
    packet.append(bpr_batt).append(",");
    packet.append(bpr_batt).append(",");
    packet.append(acc_batt).append("&");

    int i = 0;
    for (std::list<double> li : data_list) {
        if (!li.empty()) {
            double element = li.front();
            packet += std::to_string(element) += "=";
            packet += std::to_string(data[i]) + "/";
        }
        i++;                    
    }
    packet += acc_risk + "=" + "10/";
    packet += std::to_string(patient_status);
//    std::cout << packet << std::endl;
    return packet;
}

void G4T1::persistData(std::vector<std::string>& risks) {
    std::string bpr_risk, oxi_risk, ecg_risk, trm_risk;
    int id = 0;

    trm_risk = risks[0];
    ecg_risk = risks[1];
    oxi_risk = risks[2];
    bpr_risk = risks[4];
    
    fp << id++ << ",";
    fp << oxi_risk << ",";
    fp << ecg_risk << ",";
    fp << trm_risk << ",";
    fp << bpr_risk << ",";
    fp << patient_status << ",";
    fp << ((patient_status>=66)?"CRITICAL STATE":"NORMAL STATE") << ',';
    fp << std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::time_point_cast<std::chrono::milliseconds>
            (std::chrono::high_resolution_clock::now()).time_since_epoch()).count() << std::endl;
}

std::vector<std::string> G4T1::getPatientStatus() {
    std::string sensor_risk_str;
    std::string bpr;
    std::string oxi;
    std::string ecg;
    std::string trm;
    std::string acc;

    for (int i = 0; i < 4; i++) {
        double sensor_risk = data_list[i].back();


        if (sensor_risk > 0 && sensor_risk <= 20) {
            sensor_risk_str = "low risk";
        } else if (sensor_risk > 20 && sensor_risk <= 65) {
            sensor_risk_str = "moderate risk";
        } else if (sensor_risk > 65 && sensor_risk <= 100) {
            sensor_risk_str = "high risk";
        } else {
            sensor_risk_str = "unknown";
        }

        if (i==0) {
            trm = sensor_risk_str;
            trm_risk = std::to_string(sensor_risk);
        } else if (i == 1){
            ecg = sensor_risk_str;
            ecg_risk = std::to_string(sensor_risk);
        } else if (i == 2) {
            oxi = sensor_risk_str;
            oxi_risk = std::to_string(sensor_risk);
        } else if (i == 3) {
            bpr = sensor_risk_str;
            bpr_risk = std::to_string(sensor_risk);
        } else {
            acc = sensor_risk_str;
            acc_risk = std::to_string(sensor_risk);
        }
    }

    std::vector<std::string> v = {trm, ecg, oxi, bpr, acc};  
    return v;
}

void G4T1::receiveSensorData(const messages::SensorData::ConstPtr& msg) {
    std::string type = msg->type;
    double risk = msg->risk;
    double batt = msg->batt;
    std::vector<std::string> risks;
    
    std::cout << "Received data from " + type << std::endl; 

    if (type=="thermometer"){
        trm_batt = std::to_string(batt);
    } else if (type=="ecg") {
        ecg_batt = std::to_string(batt);
    } else if (type=="oximeter") {
        oxi_batt = std::to_string(batt);
    } else if (type=="bpms" || type=="bpmd") {
        bpr_batt = std::to_string(batt);
    } 

    if(type != "null" && int32_t(risk) != -1) {  
        int32_t sensor_id = get_sensor_id(type);
        data[sensor_id] = msg->data;
    
        data_list[sensor_id].push_back(risk);

        patient_status = data_fuse(data_list);

        if (connect) {
            // send data to server
        }
    }

    
    risks = getPatientStatus();
    trm_risk = risks[0];
    ecg_risk = risks[1];
    oxi_risk = risks[2];
    bpr_risk = risks[3];
    acc_risk = risks[4]; 

    if (persist)
        this->persistData(risks);

    std::cout << std::endl << "*****************************************" << std::endl;
    std::cout << "PatientStatusInfo#" << std::endl;
    std::cout << "| THERM_RISK: " << trm_risk << std::endl;
    std::cout << "| ECG_RISK: " << ecg_risk << std::endl;
    std::cout << "| OXIM_RISK: " << oxi_risk << std::endl;
    std::cout << "| BPRESS_RISK: " << bpr_risk << std::endl;
    std::cout << "| PACIENT_STATE:" << ((patient_status>=66)?"CRITICAL STATE":"NORMAL STATE") << std::endl;
    std::cout << "*****************************************" << std::endl;
}

void G4T1::reconfigure(const archlib::AdaptationCommand::ConstPtr& msg) {}

void G4T1::body() {   
    ros::NodeHandle nh;
    
    ros::Subscriber thermometerSub = nh.subscribe("thermometer_data", 10, &G4T1::receiveSensorData, this);
    ros::Subscriber oximeterSub = nh.subscribe("oximeter_data", 10, &G4T1::receiveSensorData, this);
    ros::Subscriber ecgSub = nh.subscribe("ecg_data", 10, &G4T1::receiveSensorData, this);
    ros::Subscriber diastolicSub = nh.subscribe("diastolic_data", 10, &G4T1::receiveSensorData, this);
    ros::Subscriber systolicSub = nh.subscribe("systolic_data", 10, &G4T1::receiveSensorData, this);

    ros::spin();
}
