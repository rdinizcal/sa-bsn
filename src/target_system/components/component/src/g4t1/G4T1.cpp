#include "component/g4t1/G4T1.hpp"

#define BATT_UNIT 0.001

using namespace bsn::processor;

G4T1::G4T1(int &argc, char **argv, const std::string &name) :
    CentralHub(argc, argv, name, true, bsn::resource::Battery("ch_batt", 100, 100, 1) ),
    connect(true),
    database_url(),
    lost_packt(false),
    patient_status(0.0) {}
	
G4T1::~G4T1() {}

std::string G4T1::makePacket() {
    //std::string packet = "88,97#10,20,30,40,50&";
    std::string packet = "";

    packet.append(trm_batt).append(",");
    packet.append(ecg_batt).append(",");
    packet.append(oxi_batt).append(",");
    packet.append(bpr_batt).append(",");
    packet.append(bpr_batt).append("&");

    int i = 0;
    for (std::list<double> li : data_buffer) {
        if (!li.empty()) {
            double element = li.front();
            packet += std::to_string(element) += "=";
            packet += std::to_string(li.back()) + "/";
        }
        i++;                    
    }
    packet += std::to_string(patient_status);
    return packet;
}

std::vector<std::string> G4T1::getPatientStatus() {
    std::string sensor_risk_str;
    std::string bpr;
    std::string oxi;
    std::string ecg;
    std::string trm;

    for (int i = 0; i < 4; i++) {
        double sensor_risk = data_buffer[i].back();


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
        }
    }

    std::vector<std::string> v = {trm, ecg, oxi, bpr};  
    return v;
}

void G4T1::setUp() {
    Component::setUp();

    ros::NodeHandle config;
    config.getParam("connect", connect);
    config.getParam("db_url", database_url);
    config.getParam("session", session);

    double freq;
    config.getParam("frequency", freq);
    rosComponentDescriptor.setFreq(freq);

    for(std::vector<std::list<double>>::iterator it = data_buffer.begin();
        it != data_buffer.end(); ++it) {
            (*it) = {0.0};
    }
}

void G4T1::tearDown() {}

void G4T1::collect(const messages::SensorData::ConstPtr& msg) {
    int type = get_sensor_id(msg->type);
    double risk = msg->risk;
    double batt = msg->batt;
    
    //battery.consume(BATT_UNIT);
    if(msg->type == "null" || int32_t(risk) == -1)  throw std::domain_error("risk data out of boundaries");

    /*update battery status for received sensor info*/
    if (msg->type=="thermometer"){
        trm_batt = std::to_string(batt);
    } else if (msg->type=="ecg") {
        ecg_batt = std::to_string(batt);
    } else if (msg->type=="oximeter") {
        oxi_batt = std::to_string(batt);
    } else if (msg->type=="bpms" || msg->type=="bpmd") {
        bpr_batt = std::to_string(batt);
    } 

    if(buffer_size[type] < max_size){
        data_buffer[type].push_back(risk);
        buffer_size[type] = data_buffer[type].size();
        total_buffer_size = std::accumulate(std::begin(buffer_size), std::end(buffer_size), 0, std::plus<int>());
    } else {
        data_buffer[type].push_back(risk);
        data_buffer[type].erase(data_buffer[type].begin());//erase the first element to avoid overflow
        lost_packt = true;
    }
}

void G4T1::process(){
    //battery.consume(BATT_UNIT*data_buffer.size());
    patient_status = data_fuse(data_buffer); // consumes 1 packt per sensor (in the buffers that have packages to be processed)
    for (int i = 0; i < buffer_size.size(); ++i){ // update buffer sizes
        buffer_size[i] = data_buffer[i].size();
    }
    total_buffer_size = std::accumulate(std::begin(buffer_size), std::end(buffer_size), 0, std::plus<int>()); //update total buffer size 

    std::vector<std::string> risks;
    risks = getPatientStatus();
    trm_risk = risks[0];
    ecg_risk = risks[1];
    oxi_risk = risks[2];
    bpr_risk = risks[3];

    std::cout << std::endl << "*****************************************" << std::endl;
    std::cout << "PatientStatusInfo#" << std::endl;
    std::cout << "| THERM_RISK: " << trm_risk << std::endl;
    std::cout << "| ECG_RISK: " << ecg_risk << std::endl;
    std::cout << "| OXIM_RISK: " << oxi_risk << std::endl;
    std::cout << "| BPRESS_RISK: " << bpr_risk << std::endl;
    std::cout << "| PACIENT_STATE:" << ((patient_status>=66)?"CRITICAL STATE":"NORMAL STATE") << std::endl;
    std::cout << "*****************************************" << std::endl;
    
}    

void G4T1::transfer(){
    if (connect) {
    //battery.consume(BATT_UNIT*data_buffer.size()*5);
    web::http::client::http_client client(U(database_url));
    web::json::value json_obj; 
        json_obj["VitalData"] = web::json::value::string(makePacket());
        client.request(web::http::methods::PUT, U("/sessions/" + std::to_string(session) + ".json") ,json_obj);
    }

    if(lost_packt){
        lost_packt = false;
        throw std::domain_error("lost data due to package overflow");
    }
}