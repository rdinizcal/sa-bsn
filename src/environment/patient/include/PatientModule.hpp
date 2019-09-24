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

    private:
        uint32_t getData();
        void run();

        std::map<std::string, bsn::generator::DataGenerator> patientData;

};