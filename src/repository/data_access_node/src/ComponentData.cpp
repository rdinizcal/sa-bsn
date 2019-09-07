#include "data_access_node/ComponentData.hpp"

ComponentData::ComponentData(/*int &argc, char **argv*/) {}

ComponentData::~ComponentData() {}

ComponentData::ComponentData(const ComponentData &) {}

void ComponentData::setTimestamp(double &timestamp) {
    this->timestamp = timestamp;
}

double ComponentData::getTimestamp() {
    return this->timestamp;
}

void ComponentData::setName(std::string &name) {
    this->name = name;
}

std::string ComponentData::getName() {
    return this->name;
}

void ComponentData::setType(std::string &type) {
    this->type = type;
}

std::string ComponentData::getType() {
    return this->type;
}

void ComponentData::setBatteryLevel(double &battery_level) {
    this->battery_level = battery_level;
}

double ComponentData::getBatteryLevel() {
    return this->battery_level;
}

void ComponentData::setFrequency(double &frequency) {
    this->frequency = frequency;
}

double ComponentData::getFrequency() {
    return this->frequency;
}

void ComponentData::setCost(double &cost) {
    this->cost = cost;
}

double ComponentData::getCost() {
    return this->cost;
}

void ComponentData::setRiskStatus(std::string &risk_status) {
    this->risk_status = risk_status;
}

std::string ComponentData::getRiskStatus() {
    return this->risk_status;
}

std::string ComponentData::toString() {
    std::ostringstream ss;
    std::string componentData;

    std::string separator = " ";

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