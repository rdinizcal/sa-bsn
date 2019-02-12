#include "CentralhubModule.hpp"

using namespace odcore::data;

using namespace bsn::processor;

using namespace bsn::msg::data;
using namespace bsn::msg::info;
using namespace bsn::msg::control;

using namespace bsn::communication;


CentralhubModule::CentralhubModule(const int32_t &argc, char **argv) :
TimeTriggeredConferenceClientModule(argc, argv, "centralhub"),
	buffer(),
    localQueue(),
    active(true),
    params({{"freq",1}}),
    connect(1),
    port(6060),
    ip("localhost"),
    persist(1),
    fp(),
    path("centralhub_output.csv") {}
	
CentralhubModule::~CentralhubModule() {}

void CentralhubModule::setUp() {
    addDataStoreFor(873, buffer);
    addDataStoreFor(904, buffer);

    connect = getKeyValueConfiguration().getValue<int>("centralhub.connect");
    port = getKeyValueConfiguration().getValue<int>("centralhub.port");
    ip = getKeyValueConfiguration().getValue<std::string>("centralhub.ip");
    persist = getKeyValueConfiguration().getValue<int>("centralhub.persist");
    path = getKeyValueConfiguration().getValue<std::string>("centralhub.path");

    if (persist) {
        fp.open(path);
        fp << "ID,SPO2_DATA,ECG_DATA,TEMP_DATA,BLOODPRESSURE_DATA,OVERALL_STATUS,PATIENT_STATE,TIME_MS" << endl;
    }
}

void CentralhubModule::tearDown() {
    if (persist)
        fp.close();
}

void CentralhubModule::sendTaskInfo(const std::string &task_id, const double &cost, const double &reliability, const double &frequency) {
    TaskInfo task(task_id, cost, reliability, frequency);
    Container taskContainer(task);
    getConference().send(taskContainer);
}

void CentralhubModule::sendMonitorTaskInfo(const std::string &task_id, const double &cost, const double &reliability, const double &frequency) {
    MonitorTaskInfo task(task_id, cost, reliability, frequency);
    Container taskContainer(task);
    getConference().send(taskContainer);
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode CentralhubModule::body() {
    Container container;    
    double patient_status;
    bool received = false;
    std::string packet;
    std::array<double, 5> data;
    int id = 0;
    double reli = 1;

    TCPSend sender;
    if (connect) {
        sender.set_port(port);
        sender.setIP(ip);
        sender.connect();
    }

    vector<list<double>> data_list(5);
    for(std::vector<std::list<double>>::iterator it = data_list.begin();
        it != data_list.end(); ++it) {
            (*it).push_back(0.0);
    }

    while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
        
        received = false;
        cout << "operating frequency: " << params["freq"] << endl;

        while(!buffer.isEmpty()){
            
            container = buffer.leave();

            if(container.getDataType()==904){
                active = container.getData<CentralHubControlCommand>().getActive();
                params["freq"] = container.getData<CentralHubControlCommand>().getFrequency(); 
            }
            if(container.getDataType()==873){
                Container ncont(container.getData<SensorData>());
                localQueue.add(ncont);
            }
        }
        
        /*sets the centralhub reliability based on the number of messages to process (fibonacci)*/
        if(!localQueue.isEmpty()){
            std::cout << "localQueue size:" << localQueue.getSize() << std::endl;

            if (localQueue.getSize()>8) {
                reli = 0.95 - 0.03*13;
            } else if (localQueue.getSize()>6) {
                reli = 0.95 - 0.03*5;
            } else if (localQueue.getSize()>4) {
                reli = 0.95 - 0.03*3;
            } else if (localQueue.getSize()>2) {
                reli = 0.95 - 0.03*2;
            } else {
                reli = 0.95;
            }

            { // update controller with task info
                sendTaskInfo("G4_T1.1",0,1,params["freq"]);
                sendTaskInfo("G4_T1.2",0,1,params["freq"]);
                sendTaskInfo("G4_T1.3",0,1,params["freq"]);
            // and the monitor..
                sendMonitorTaskInfo("G4_T1.1",0,1,params["freq"]);
                sendMonitorTaskInfo("G4_T1.2",0,reli,params["freq"]);
                sendMonitorTaskInfo("G4_T1.3",0,1,params["freq"]);
            }
        }

        if((rand() % 100)+1 < int32_t(params["freq"]*100)){
            
            while(!localQueue.isEmpty()){

                container = localQueue.leave();

                std::string type = container.getData<SensorData>().getType();
                double risk = container.getData<SensorData>().getRisk();
                if(type=="null"){ continue; } // for joker packages

                int32_t sensor_id = get_sensor_id(type);
                data[sensor_id] = container.getData<SensorData>().getData();

                if (int32_t(risk) == -1) { continue; }
                data_list[sensor_id].push_back(risk);

                received=true;
            }
            
            if (!received) { continue; }
            patient_status = data_fuse(data_list);

            {  // send data to the server
                if (connect) {
                    packet = "";
                    int i = 0;
                    for (list<double> li : data_list) {
                        if (!li.empty()) {
                            double element = li.front();
                            packet += to_string(element) += "=";
                            packet += to_string(data[i]) + "/";
                        }
                        i++;                    
                    }
                    packet += to_string(patient_status);
                    sender.send(packet);
                }
            }

            { // Persist and send data to controller
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

                if (persist) {
                    fp << id++ << ",";
                    fp << oxi_risk << ",";
                    fp << ecg_risk << ",";
                    fp << trm_risk << ",";
                    fp << bpr_risk << ",";
                    fp << patient_status << ",";
                    fp << ((patient_status>=66)?"CRITICAL STATE":"NORMAL STATE") << ',';
                    fp << std::chrono::duration_cast<std::chrono::milliseconds>
                            (std::chrono::time_point_cast<std::chrono::milliseconds>
                            (std::chrono::high_resolution_clock::now()).time_since_epoch()).count() << endl;
                }
                
                ContextInfo contextInfo("patient health status", false, 0, 0, (patient_status>=66)?"CRITICAL STATE":"NORMAL STATE");
                Container contextInfoContainer(contextInfo);
                getConference().send(contextInfoContainer);

                cout << endl << "*****************************************" << endl;
                cout << "PatientStatusInfo#" << endl;
                cout << "| THERM_RISK: " << trm_risk << endl;
                cout << "| ECG_RISK: " << ecg_risk << endl;
                cout << "| OXIM_RISK: " << oxi_risk << endl;
                cout << "| BPRESS_RISK: " << bpr_risk << endl;
                cout << "| PACIENT_STATE:" << ((patient_status>=66)?"CRITICAL STATE":"NORMAL STATE") << endl;
                cout << "*****************************************" << endl;
            }
        }

    }

    return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}
