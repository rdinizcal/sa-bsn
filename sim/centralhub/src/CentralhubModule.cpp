#include "CentralhubModule.hpp"

using namespace bsn::processor;

CentralhubModule::CentralhubModule(const int32_t &argc, char **argv) :
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
	
CentralhubModule::~CentralhubModule() {}

void CentralhubModule::setUp() {
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
}

void CentralhubModule::tearDown() {
    if (persist)
        fp.close();
}

void CentralhubModule::sendTaskInfo(const std::string &task_id, const double &cost, const double &reliability, const double &frequency) {
    // TaskInfo task(task_id, cost, reliability, frequency);
    // Container taskContainer(task);
    // getConference().send(taskContainer);
}

void CentralhubModule::sendMonitorTaskInfo(const std::string &task_id, const double &cost, const double &reliability, const double &frequency) {
    // MonitorTaskInfo task(task_id, cost, reliability, frequency);
    // Container taskContainer(task);
    // getConference().send(taskContainer);
}

std::string CentralhubModule::makePacket() {
    std::string packet = "88,97#10,20,30,40,50&";
    int i = 0;
    for (std::list<double> li : data_list) {
        if (!li.empty()) {
            double element = li.front();
            packet += std::to_string(element) += "=";
            packet += std::to_string(data[i]) + "/";
        }
        i++;                    
    }
    packet += std::to_string(patient_status);
    return packet;
}

void CentralhubModule::persistData(std::vector<std::string>& risks) {
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

std::vector<std::string> CentralhubModule::getPatientStatus() {
    std::string sensor_risk_str;
    std::string bpr_risk;
    std::string oxi_risk;
    std::string ecg_risk;
    std::string trm_risk;

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
            trm_risk = sensor_risk_str;
        } else if (i == 1){
            ecg_risk = sensor_risk_str;
        } else if (i == 2) {
            oxi_risk = sensor_risk_str;
        } else {
            bpr_risk = sensor_risk_str;
        }
    }

    std::vector<std::string> v = {trm_risk, ecg_risk, oxi_risk, bpr_risk};  
    return v;
}

void CentralhubModule::receiveSensorData(const bsn::SensorData::ConstPtr& msg) {
    std::string type = msg->type;
    double risk = msg->risk;
    std::string bpr_risk, oxi_risk, ecg_risk, trm_risk;
    std::vector<std::string> risks;
    web::http::client::http_client client(U(database_url));
    web::json::value json_obj; 

    std::cout << "Received data from " + type << std::endl; 
    
    if(type != "null" && int32_t(risk) != -1) {  
        int32_t sensor_id = get_sensor_id(type);
        data[sensor_id] = msg->data;
    
        data_list[sensor_id].push_back(risk);

        patient_status = data_fuse(data_list);

        if (connect) {
            json_obj["data"] = web::json::value::string(makePacket());
            client.request(web::http::methods::PUT, U("/sessions/" + std::to_string(session) + ".json") ,json_obj);
        }
    }

    risks = getPatientStatus();
    trm_risk = risks[0];
    ecg_risk = risks[1];
    oxi_risk = risks[2];
    bpr_risk = risks[3];
    
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

void CentralhubModule::run() {   
    bool received = false;
    std::string packet;
    // std::array<double, 5> data;
    int id = 0;
    double reli = 1;
    ros::NodeHandle nh;
    
    ros::Subscriber thermometerSub = nh.subscribe("thermometer_data", 10, &CentralhubModule::receiveSensorData, this);
    ros::Subscriber oximeterSub = nh.subscribe("oximeter_data", 10, &CentralhubModule::receiveSensorData, this);
    ros::Subscriber ecgSub = nh.subscribe("ecg_data", 10, &CentralhubModule::receiveSensorData, this);
    ros::Subscriber diastolicSub = nh.subscribe("diastolic_data", 10, &CentralhubModule::receiveSensorData, this);
    ros::Subscriber systolicSub = nh.subscribe("systolic_data", 10, &CentralhubModule::receiveSensorData, this);

    ros::spin();
    // while (ros::ok()) {
        
    //     received = false;
    //     cout << "operating frequency: " << params["freq"] << endl;

        // while(!buffer.isEmpty()){
            
        //     container = buffer.leave();

        //     if(container.getDataType()==904){
        //         active = container.getData<CentralHubControlCommand>().getActive();
        //         params["freq"] = container.getData<CentralHubControlCommand>().getFrequency(); 
        //     }
        //     if(container.getDataType()==873){
        //         Container ncont(container.getData<SensorData>());
        //         localQueue.add(ncont);
        //     }
        // }
        
        /*sets the centralhub reliability based on the number of messages to process (fibonacci)*/
        // if(!localQueue.isEmpty()){
        //     std::cout << "localQueue size:" << localQueue.getSize() << std::endl;

        //     if (localQueue.getSize()>8) {
        //         reli = 0.95 - 0.03*13;
        //     } else if (localQueue.getSize()>6) {
        //         reli = 0.95 - 0.03*5;
        //     } else if (localQueue.getSize()>4) {
        //         reli = 0.95 - 0.03*3;
        //     } else if (localQueue.getSize()>2) {
        //         reli = 0.95 - 0.03*2;
        //     } else {
        //         reli = 0.95;
        //     }

            // { // update controller with task info
            //     sendTaskInfo("G4_T1.1",0,1,params["freq"]);
            //     sendTaskInfo("G4_T1.2",0,1,params["freq"]);
            //     sendTaskInfo("G4_T1.3",0,1,params["freq"]);
            // // and the monitor..
            //     sendMonitorTaskInfo("G4_T1.1",0,1,params["freq"]);
            //     sendMonitorTaskInfo("G4_T1.2",0,reli,params["freq"]);
            //     sendMonitorTaskInfo("G4_T1.3",0,1,params["freq"]);
            // }
        // }

        // if((rand() % 100)+1 < int32_t(params["freq"]*100)){
            
        //  

        //     { // Persist and send data to controller
                     
                 // ContextInfo contextInfo("patient health status", false, 0, 0, (patient_status>=66)?"CRITICAL STATE":"NORMAL STATE");
        //         // Container contextInfoContainer(contextInfo);
        //         // getConference().send(contextInfoContainer);
                // }
    

    // }

    return;
}
