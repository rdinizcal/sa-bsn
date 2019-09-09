#include "data_access_node/ComponentData.hpp"

ComponentData::ComponentData(/*int &argc, char **argv*/) {}

ComponentData::~ComponentData() {}

ComponentData::ComponentData(const ComponentData &obj) :
    timestamp(obj.getTimestamp()),
    name(obj.getName()),
    type(obj.getType()),
    battery_level(obj.getBatteryLevel()),
    frequency(obj.getFrequency()),
    cost(obj.getCost()),
    risk_status(obj.getRiskStatus()) {}

void ComponentData::setTimestamp(double &timestamp) {
    this->timestamp = timestamp;
}

double ComponentData::getTimestamp() const {
    return this->timestamp;
}

void ComponentData::setName(std::string &name) {
    this->name = name;
}

std::string ComponentData::getName() const {
    return this->name;
}

void ComponentData::setType(std::string &type) {
    this->type = type;
}

std::string ComponentData::getType() const {
    return this->type;
}

void ComponentData::setBatteryLevel(double &battery_level) {
    this->battery_level = battery_level;
}

double ComponentData::getBatteryLevel() const {
    return this->battery_level;
}

void ComponentData::setFrequency(double &frequency) {
    this->frequency = frequency;
}

double ComponentData::getFrequency() const {
    return this->frequency;
}

void ComponentData::setCost(double &cost) {
    this->cost = cost;
}

double ComponentData::getCost() const {
    return this->cost;
}

void ComponentData::setRiskStatus(std::string &risk_status) {
    this->risk_status = risk_status;
}

std::string ComponentData::getRiskStatus() const {
    return this->risk_status;
}

std::string ComponentData::toString() {
    std::ostringstream ss;
    std::string componentData;

    std::string separator = ",";

    ss << this->getTimestamp();
    componentData += ss.str() + separator;

    ss.str(std::string());

    componentData += this->getName() + separator;
    componentData += this->getType() + separator;

    ss << this->getBatteryLevel();
    componentData += ss.str() + separator;

    ss.str(std::string());

    ss << this->getFrequency();
    componentData += ss.str() + separator;

    ss.str(std::string());

    ss << this->getCost();
    componentData += ss.str() + separator;

    componentData += this->getRiskStatus() + "\n";

    return componentData;
}