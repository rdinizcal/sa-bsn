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
        bool getPatientData();
        bsn::generator::DataGenerator configureDataGenerator(const std::string& vitalSign);

        std::map<std::string, bsn::generator::DataGenerator> patientData;
};