#include "component/Sensor.hpp"


Sensor::Sensor(int &argc, char **argv, const std::string &name, const std::string &type, const bool &active, const double &accuracy, const bsn::resource::Battery &battery) : Component(argc, argv, name), type(type), active(active), accuracy(accuracy), battery(battery), data(0.0) {}

Sensor::~Sensor() {}

Sensor& Sensor::operator=(const Sensor &obj) {
    this->type = obj.type;
    this->active = obj.active;
    this->accuracy = obj.accuracy;
    this->battery = obj.battery;
    this->data = obj.data;
}

void Sensor::body() {
    
    if (!isActive() && battery.getCurrentLevel() > 90){
        turnOn();
    } else if (isActive() && battery.getCurrentLevel() < 2){
        turnOff();        
    }

    if(isActive()) {
        data = Sensor::collect();
        data = Sensor::process(data);
        Sensor::transfer(data);
    } else {
        recharge();
    }
}

void Sensor::reconfigure(const archlib::AdaptationCommand::ConstPtr& msg) {
    std::string action = msg->action.c_str();

    char *buffer = strdup(action.c_str());
    char *pair = strtok(buffer, ",");
    char *key = strtok(pair, "=");
    double value  = std::stod(strtok(NULL, "="));

    if(key=="freq"){
        rosComponentDescriptor.setFreq(value);
    }
}

double Sensor::collect() {
    double x = 0;
    x = collect();
    return x;
}

double Sensor::process(const double &data) {
    double x = 0;
    x = process(data);
    return x;
}

void Sensor::transfer(const double &data) {
    transfer(data);
}

bool Sensor::isActive() {
    return active;
}

void Sensor::turnOn() {
    active = true;
    activate();
}

void Sensor::turnOff() {
    active = false;
    deactivate();
}

void Sensor::recharge() {
    if(battery.getCurrentLevel() <= 100) battery.generate(2.5);
}