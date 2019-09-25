#include <PatientModule.hpp>

PatientModule::PatientModule() {}

PatientModule::~PatientModule() {}

void PatientModule::setUp() {
    srand(time(NULL));

    // TODO change Operation to static namespace
    bsn::operation::Operation op;
    std::string vitalSigns;
    ros::NodeHandle handle;

    // Get what vital signs this module will simulate
    handle.getParam("vitalSigns", vitalSigns); 

    // Removes white spaces from vitalSigns
    std::remove_if(vitalSigns.begin(), vitalSigns.end(), isspace);

    std::vector<std::string> splittedVitalSigns = op.split(vitalSigns, ',');

    // For each vital sign, build its data generator
    for (const std::string& s : splittedVitalSigns) {
        patientData[s] = configureDataGenerator(s);
    }
}

bsn::generator::DataGenerator PatientModule::configureDataGenerator(const std::string& vitalSign) {
    srand(time(NULL));

    bsn::operation::Operation op;    
    
    std::vector<std::string> t_probs;
    std::array<float, 25> transitions;
    std::array<bsn::range::Range,5> ranges;
    std::string s;
    ros::NodeHandle handle;

    for(uint32_t i = 0; i < transitions.size(); i++){
        for(uint32_t j = 0; j < 5; j++){
            handle.getParam(vitalSign + "State" + std::to_string(j), s);
            t_probs = op.split(s, ',');
            for(uint32_t k = 0; k < 5; k++){
                transitions[i++] = std::stod(t_probs[k]);
            }
        }
    }
    
    std::vector<std::string> lrs,mrs0,hrs0,mrs1,hrs1;

    handle.getParam(vitalSign + "LowRisk", s);
    lrs = op.split(s, ',');
    handle.getParam(vitalSign + "MidRisk0", s);
    mrs0 = op.split(s, ',');
    handle.getParam(vitalSign + "HighRisk0", s);
    hrs0 = op.split(s, ',');
    handle.getParam(vitalSign + "MidRisk1", s);
    mrs1 = op.split(s, ',');
    handle.getParam(vitalSign + "HighRisk1", s);
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

uint32_t PatientModule::getData(const std::string& vitalSign) {
    // TODO return the specified data with patientData
    return patientData[vitalSign].getValue();
}

void PatientModule::run() {
    // TODO client-server logic to get the patient data
}

