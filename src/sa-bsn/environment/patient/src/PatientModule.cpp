#include <PatientModule.hpp>

PatientModule::PatientModule(int  &argc, char **argv, std::string name) : ROSComponent(argc, argv, name) {}

PatientModule::~PatientModule() {}

void PatientModule::setUp() {
    srand(time(NULL));

    // TODO change Operation to static
    std::string vitalSigns;
    service = nh.advertiseService("getPatientData", &PatientModule::getPatientData, this);
    double aux;

    frequency = 1000;

    // Get what vital signs this module will simulate
    nh.getParam("vitalSigns", vitalSigns);

    // Removes white spaces from vitalSigns
    vitalSigns.erase(std::remove(vitalSigns.begin(), vitalSigns.end(),' '), vitalSigns.end());

    std::vector<std::string> splittedVitalSigns = bsn::utils::split(vitalSigns, ',');

    for (std::string s : splittedVitalSigns) {
        vitalSignsFrequencies[s] = 0;
        nh.getParam(s + "_Change", aux);
        vitalSignsChanges[s] = 1/aux;
        nh.getParam(s + "_Offset", vitalSignsOffsets[s]);
    }

    for (const std::string& s : splittedVitalSigns) {
        patientData[s] = configureDataGenerator(s);
    }

    rosComponentDescriptor.setFreq(frequency);
    
    period = 1/frequency;

    dynamic_reconfigure::Server<patient::PatientConfig>::CallbackType f;
    f = boost::bind(&PatientModule::callback, this, _1, _2);
    server.setCallback(f);
}

bsn::generator::DataGenerator PatientModule::configureDataGenerator(const std::string& vitalSign) {
    srand(time(NULL));
    
    std::vector<std::string> t_probs;
    std::array<float, 25> transitions;
    std::array<bsn::range::Range,5> ranges;
    std::string s;
    ros::NodeHandle handle;

    // std::cout << vitalSign << std::endl;
    for(uint32_t i = 0; i < transitions.size(); i++){
        for(uint32_t j = 0; j < 5; j++){
            handle.getParam(vitalSign + "_State" + std::to_string(j), s);
            t_probs = bsn::utils::split(s, ',');
            for(uint32_t k = 0; k < 5; k++){
                transitions[i++] = std::stod(t_probs[k]);
            }
        }
    }
    
    std::vector<std::string> lrs,mrs0,hrs0,mrs1,hrs1;

    handle.getParam(vitalSign + "_LowRisk", s);
    lrs = bsn::utils::split(s, ',');
    handle.getParam(vitalSign + "_MidRisk0", s);
    mrs0 = bsn::utils::split(s, ',');
    handle.getParam(vitalSign + "_HighRisk0", s);
    hrs0 = bsn::utils::split(s, ',');
    handle.getParam(vitalSign + "_MidRisk1", s);
    mrs1 = bsn::utils::split(s, ',');
    handle.getParam(vitalSign + "_HighRisk1", s);
    hrs1 = bsn::utils::split(s, ',');

    ranges[0] = bsn::range::Range(std::stod(hrs0[0]), std::stod(hrs0[1]));
    ranges[1] = bsn::range::Range(std::stod(mrs0[0]), std::stod(mrs0[1]));
    ranges[2] = bsn::range::Range(std::stod(lrs[0]), std::stod(lrs[1]));
    ranges[3] = bsn::range::Range(std::stod(mrs1[0]), std::stod(mrs1[1]));
    ranges[4] = bsn::range::Range(std::stod(hrs1[0]), std::stod(hrs1[1]));

    bsn::generator::Markov markov(transitions, ranges, 2);
    bsn::generator::DataGenerator dataGenerator(markov);
    dataGenerator.setSeed();

    std::cout << "Setting DataGenerator of: " << vitalSign << std::endl;    
    std::cout << "HighRisk0: " << hrs0[0] << " to " << hrs0[1] << std::endl; 
    std::cout << "MidRisk0 : " << mrs0[0] << " to " << mrs0[1] << std::endl; 
    std::cout << "LowRisk  : " << lrs[0]  << " to " << lrs[1] << std::endl; 
    std::cout << "MidRisk1 : " << mrs1[0] << " to " << mrs1[1] << std::endl; 
    std::cout << "HighRisk1: " << hrs1[0] << " to " << hrs1[1] << std::endl; 
    
    return dataGenerator;
}

void PatientModule::tearDown() {}

bool PatientModule::getPatientData(services::PatientData::Request &request, 
                                services::PatientData::Response &response) {
    
    response.data = patientData[request.vitalSign].getValue();
    
    std::cout << "Send " + request.vitalSign + " data." << std::endl;

    return true;
}

void PatientModule::body() {
    for (auto &p : vitalSignsFrequencies) {
        
        if (p.second >= (vitalSignsChanges[p.first] + vitalSignsOffsets[p.first])) {
            patientData[p.first].nextState();
            p.second = vitalSignsOffsets[p.first];
            std::cout << "Changed " + p.first + " state." << std::endl;
        } else {
            p.second += period;
        }
        
    }
}

void PatientModule::callback(patient::PatientConfig &config, uint32_t level){

    ros::NodeHandle handle;

    handle.setParam("oxigenation_LowRisk", config.oxigenation_LowRisk.c_str());
    handle.setParam("oxigenation_MidRisk0", config.oxigenation_MidRisk0.c_str());
    handle.setParam("oxigenation_HighRisk0", config.oxigenation_HighRisk0.c_str());
    handle.setParam("oxigenation_MidRisk1", config.oxigenation_MidRisk1.c_str());
    handle.setParam("oxigenation_HighRisk1", config.oxigenation_HighRisk1.c_str());

    handle.setParam("oxigenation_State0", config.oxigenation_State0.c_str());
    handle.setParam("oxigenation_State1", config.oxigenation_State1.c_str());
    handle.setParam("oxigenation_State2", config.oxigenation_State2.c_str());
    handle.setParam("oxigenation_State3", config.oxigenation_State3.c_str());
    handle.setParam("oxigenation_State4", config.oxigenation_State4.c_str());


    handle.setParam("heart_rate_LowRisk", config.heart_rate_LowRisk.c_str());
    handle.setParam("heart_rate_MidRisk0", config.heart_rate_MidRisk0.c_str());
    handle.setParam("heart_rate_HighRisk0", config.heart_rate_HighRisk0.c_str());
    handle.setParam("heart_rate_MidRisk1", config.heart_rate_MidRisk1.c_str());
    handle.setParam("heart_rate_HighRisk1", config.heart_rate_HighRisk1.c_str());

    handle.setParam("heart_rate_State0", config.heart_rate_State0.c_str());
    handle.setParam("heart_rate_State1", config.heart_rate_State1.c_str());
    handle.setParam("heart_rate_State2", config.heart_rate_State2.c_str());
    handle.setParam("heart_rate_State3", config.heart_rate_State3.c_str());
    handle.setParam("heart_rate_State4", config.heart_rate_State4.c_str());


    handle.setParam("temperature_LowRisk", config.temperature_LowRisk.c_str());
    handle.setParam("temperature_MidRisk0", config.temperature_MidRisk0.c_str());
    handle.setParam("temperature_HighRisk0", config.temperature_HighRisk0.c_str());
    handle.setParam("temperature_MidRisk1", config.temperature_MidRisk1.c_str());
    handle.setParam("temperature_HighRisk1", config.temperature_HighRisk1.c_str());

    handle.setParam("temperature_State0", config.temperature_State0.c_str());
    handle.setParam("temperature_State1", config.temperature_State1.c_str());
    handle.setParam("temperature_State2", config.temperature_State2.c_str());
    handle.setParam("temperature_State3", config.temperature_State3.c_str());
    handle.setParam("temperature_State4", config.temperature_State4.c_str());


    handle.setParam("abpd_LowRisk", config.abpd_LowRisk.c_str());
    handle.setParam("abpd_MidRisk0", config.abpd_MidRisk0.c_str());
    handle.setParam("abpd_HighRisk0", config.abpd_HighRisk0.c_str());
    handle.setParam("abpd_MidRisk1", config.abpd_MidRisk1.c_str());
    handle.setParam("abpd_HighRisk1", config.abpd_HighRisk1.c_str());

    handle.setParam("abpd_State0", config.abpd_State0.c_str());
    handle.setParam("abpd_State1", config.abpd_State1.c_str());
    handle.setParam("abpd_State2", config.abpd_State2.c_str());
    handle.setParam("abpd_State3", config.abpd_State3.c_str());
    handle.setParam("abpd_State4", config.abpd_State4.c_str());

    handle.setParam("abps_LowRisk", config.abps_LowRisk.c_str());
    handle.setParam("abps_MidRisk0", config.abps_MidRisk0.c_str());
    handle.setParam("abps_HighRisk0", config.abps_HighRisk0.c_str());
    handle.setParam("abps_MidRisk1", config.abps_MidRisk1.c_str());
    handle.setParam("abps_HighRisk1", config.abps_HighRisk1.c_str());

    handle.setParam("abps_State0", config.abps_State0.c_str());
    handle.setParam("abps_State1", config.abps_State1.c_str());
    handle.setParam("abps_State2", config.abps_State2.c_str());
    handle.setParam("abps_State3", config.abps_State3.c_str());
    handle.setParam("abps_State4", config.abps_State4.c_str());


    handle.setParam("glucose_LowRisk", config.glucose_LowRisk.c_str());
    handle.setParam("glucose_MidRisk0", config.glucose_MidRisk0.c_str());
    handle.setParam("glucose_HighRisk0", config.glucose_HighRisk0.c_str());
    handle.setParam("glucose_MidRisk1", config.glucose_MidRisk1.c_str());
    handle.setParam("glucose_HighRisk1", config.glucose_HighRisk1.c_str());

    handle.setParam("glucose_State0", config.glucose_State0.c_str());
    handle.setParam("glucose_State1", config.glucose_State1.c_str());
    handle.setParam("glucose_State2", config.glucose_State2.c_str());
    handle.setParam("glucose_State3", config.glucose_State3.c_str());
    handle.setParam("glucose_State4", config.glucose_State4.c_str());

    patientData["oxigenation"] = this->configureDataGenerator("oxigenation");
    patientData["heart_rate"] = this->configureDataGenerator("heart_rate");
    patientData["temperature"] = this->configureDataGenerator("temperature");
    patientData["abpd"] = this->configureDataGenerator("abpd");
    patientData["abps"] = this->configureDataGenerator("abps");
    patientData["glucose"] = this->configureDataGenerator("glucose");

}