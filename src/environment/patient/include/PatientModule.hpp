#include <ros/ros.h>
#include "bsn/generator/DataGenerator.hpp"
#include "bsn/operation/Operation.hpp"
#include <string>

class PatientModule {
    public:
        PatientModule();
        ~PatientModule();

        void setUp();
        void tearDown();
        void run();

    private:
        uint32_t getData(const std::string& vitalSign);
        bsn::generator::DataGenerator configureDataGenerator(const std::string& vitalSign);

        std::map<std::string, bsn::generator::DataGenerator> patientData;
};