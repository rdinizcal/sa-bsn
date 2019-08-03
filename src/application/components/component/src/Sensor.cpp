#include "component/Sensor.hpp"


Sensor::Sensor(const int32_t &argc, char **argv, const std::string &type, const bool &active, const double &accuracy, const bsn::resource::Battery &battery) : SchedulableComponent(argc, argv), type(type), active(active), accuracy(accuracy), battery(battery), data(0.0) {}

Sensor::~Sensor() {}

Sensor& Sensor::operator=(const Sensor &obj) {
    this->type = obj.type;
    this->active = obj.active;
    this->accuracy = obj.accuracy;
    this->battery = obj.battery;
    this->data = obj.data;
}

void Sensor::sendEvent(const std::string &type, const std::string &description) {
    messages::Event msg;

    msg.source = ros::this_node::getName();
    msg.type = type;
    msg.description = description;

    event_pub = handle.advertise<messages::Event>("collect_event", 10);
    event_pub.publish(msg);
}

void Sensor::sendStatus(const std::string &id, const double &value) {
    messages::Status msg;

    msg.source = ros::this_node::getName();
    msg.key = id;
    msg.value = value;

    status_pub = handle.advertise<messages::Status>("collect_status", 10);
    status_pub.publish(msg);
}

void Sensor::body() {
    
    if (!isActive() && battery.getCurrentLevel() > 90){
        turnOn();
        sendStatus(ros::this_node::getName(), active?1:0);
    } else if (isActive() && battery.getCurrentLevel() < 2){
        turnOff();
        sendStatus(ros::this_node::getName(), active?1:0);
    }

    if(isActive()) {
        data = Sensor::collect();
        data = Sensor::process(data);
        Sensor::transfer(data);
    } else {
        recharge();
    }

}

double Sensor::collect() {
    double x = 0;

    try {
        x = collect();
    } catch (const std::exception& e) {
        sendEvent(e.what(), "collected data exceeded range");
    }

    return x;
}

double Sensor::process(const double &data) {
    double x = 0;

    try {
        x = process(data);
    } catch (const std::exception& e) {
        if(e.what() == "failure") sendEvent(e.what(), "could not process data");
        else std::cout << e.what();
    }

    return x;
}

void Sensor::transfer(const double &data) {
    
    try {
        transfer(data);
    } catch (const std::exception& e) {
        if(e.what() == "failure") sendEvent(e.what(), "failed at transfering data");
        else std::cout << e.what();
    }

}

bool Sensor::isActive() {
    return active;
}

void Sensor::turnOn() {
    active = true;
}

void Sensor::turnOff() {
    active = false;
}

void Sensor::recharge() {
    if(battery.getCurrentLevel() <= 100) battery.generate(2.5);
}