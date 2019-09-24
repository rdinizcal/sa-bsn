#include <PatientModule.hpp>

PatientModule::PatientModule() {}

PatientModule::~PatientModule() {}

void PatientModule::setUp() {
    srand(time(NULL));

    bsn::operation::Operation op;

    // Configure temperature
    patientData["temperature"] = configureDataGenerator();

    // Configure oxigenation
    patientData["oxigenation"] = configureDataGenerator();

    // Configure heart frequency
    patientData["heart"] = configureDataGenerator();

    // Configure systolic pressure
    patientData["systolic"] = configureDataGenerator();

    // Configure dyastolic pressure
    patientData["diastolic"] = configureDataGenerator();
}

bsn::generator::DataGenerator configureDataGenerator() {
    // TODO implement this configuration for 3 and 5 cases
}

void PatientModule::tearDown() {}

uint32_t PatientModule::getData() {
    // TODO return the specified data with patientData
}

void PatientModule::run() {
    // TODO client-server logic to get the patient data
}

