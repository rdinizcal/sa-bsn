#include <PatientModule.hpp>

PatientModule::PatientModule(int  &argc, char **argv, std::string name) : ROSComponent(argc, argv, name) {}

PatientModule::~PatientModule() {}

void PatientModule::setUp() {
    srand(time(NULL));

    // TODO change Operation to static namespace
    bsn::operation::Operation op;
    std::string vitalSigns;
    ros::NodeHandle handle;
    double aux;

    // Get what vital signs this module will simulate
    nh.getParam("vitalSigns", vitalSigns);
    nh.getParam("frequency", frequency);

    // Removes white spaces from vitalSigns
    vitalSigns.erase(std::remove(vitalSigns.begin(), vitalSigns.end(),' '), vitalSigns.end());

    std::vector<std::string> splittedVitalSigns = op.split(vitalSigns, ',');

    for (std::string s : splittedVitalSigns) {
        vitalSignsFrequencies[s] = 0;
        nh.getParam(s + "_Change", aux);
        frequency = std::max(frequency, aux);
        vitalSignsChanges[s] = 1/aux;
        nh.getParam(s + "_Offset", vitalSignsOffsets[s]);
    }

    // For each vital sign, build its data generator
    for (const std::string& s : splittedVitalSigns) {
        patientData[s] = configureDataGenerator(s);
    }
    rosComponentDescriptor.setFreq(frequency);
}

bsn::generator::DataGenerator PatientModule::configureDataGenerator(const std::string& vitalSign) {
    srand(time(NULL));

    bsn::operation::Operation op;    
    
    std::vector<std::string> t_probs;
    std::array<float, 25> transitions;
    std::array<bsn::range::Range,5> ranges;
    std::string s;
    ros::NodeHandle handle;

    // std::cout << vitalSign << std::endl;
    for(uint32_t i = 0; i < transitions.size(); i++){
        for(uint32_t j = 0; j < 5; j++){
            handle.getParam(vitalSign + "_State" + std::to_string(j), s);
            t_probs = op.split(s, ',');
            for(uint32_t k = 0; k < 5; k++){
                transitions[i++] = std::stod(t_probs[k]);
            }
        }
    }
    
    std::vector<std::string> lrs,mrs0,hrs0,mrs1,hrs1;

    handle.getParam(vitalSign + "_LowRisk", s);
    lrs = op.split(s, ',');
    handle.getParam(vitalSign + "_MidRisk0", s);
    mrs0 = op.split(s, ',');
    handle.getParam(vitalSign + "_HighRisk0", s);
    hrs0 = op.split(s, ',');
    handle.getParam(vitalSign + "_MidRisk1", s);
    mrs1 = op.split(s, ',');
    handle.getParam(vitalSign + "_HighRisk1", s);
    hrs1 = op.split(s, ',');

    ranges[0] = bsn::range::Range(std::stod(hrs0[0]), std::stod(hrs0[1]));
    ranges[1] = bsn::range::Range(std::stod(mrs0[0]), std::stod(mrs0[1]));
    ranges[2] = bsn::range::Range(std::stod(lrs[0]), std::stod(lrs[1]));
    ranges[3] = bsn::range::Range(std::stod(mrs1[0]), std::stod(mrs1[1]));
    ranges[4] = bsn::range::Range(std::stod(hrs1[0]), std::stod(hrs1[1]));

    bsn::generator::Markov markov(transitions, ranges, 2);
    bsn::generator::DataGenerator dataGenerator(markov);
    dataGenerator.setSeed();

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
    ros::ServiceServer service = nh.advertiseService("getPatientData", &PatientModule::getPatientData, this);

    period = 1/frequency;

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

