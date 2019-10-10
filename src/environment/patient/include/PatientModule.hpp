#include <ros/ros.h>
#include "bsn/generator/DataGenerator.hpp"
#include "bsn/operation/Operation.hpp"
#include "bsn/range/Range.hpp"
#include <string>
#include "services/PatientData.h"

class PatientModule {
    public:
        PatientModule();
        ~PatientModule();

        void setUp();
        void tearDown();
        void run();

    private:
        bool getPatientData(services::PatientData::Request &request, services::PatientData::Response &response);
        bsn::generator::DataGenerator configureDataGenerator(const std::string& vitalSign);

        std::map<std::string, bsn::generator::DataGenerator> patientData;
        std::map<std::string, double> vitalSignsFrequencies;
        std::map<std::string, double> vitalSignsChanges;
        std::map<std::string, double> vitalSignsOffsets;

        double frequency;
};