#include "controller/CurrentInfo.hpp"

CurrentInfo::CurrentInfo(const std::string &module_name, const float &expected_battery_life, 
                            const float &battery_level, const float &cost, const float &frequency, const std::string &risk_status) : 
	module_name(module_name),
    expected_battery_life(expected_battery_life),
    battery_level(battery_level),
    cost(cost),
    frequency(frequency),
    risk_status(risk_status) {}

CurrentInfo::CurrentInfo(const CurrentInfo &obj) :
    module_name(obj.getModuleName()),
    expected_battery_life(obj.getExpectedBatteryLife()),
    battery_level(obj.getBatteryLevel()),
    cost(obj.getCost()),
    frequency(obj.getFrequency()),
    risk_status(obj.getRiskStatus())
	{}

CurrentInfo& CurrentInfo::operator=(const CurrentInfo &obj) {
    this->module_name = obj.getModuleName();
    this->expected_battery_life = obj.getExpectedBatteryLife();
    this->battery_level = obj.getBatteryLevel();
    this->cost = obj.getCost();
    this->frequency = obj.getFrequency();
    this->risk_status = obj.getRiskStatus();

    return (*this);
}

CurrentInfo::CurrentInfo() : 
	module_name(""),
    expected_battery_life(0),
    battery_level(0),
    cost(0),
    frequency(0),
    risk_status("") {}

CurrentInfo::~CurrentInfo() {}

void CurrentInfo::setModuleName(const std::string &module_name){
    this->module_name = module_name;
}

std::string CurrentInfo::getModuleName() const {
    return this->module_name;
}

void CurrentInfo::setExpectedBatteryLife(const float &expected_battery_life) {
    this->expected_battery_life = expected_battery_life;
}

float CurrentInfo::getExpectedBatteryLife() const {
    return this->expected_battery_life;
}

void CurrentInfo::setBatteryLevel(const float &battery_level) {
    this->battery_level = battery_level;
}

float CurrentInfo::getBatteryLevel() const {
    return this->battery_level;
}

void CurrentInfo::setCost(const float &cost) {
    this->cost = cost;
}

float CurrentInfo::getCost() const {
    return this->cost;
}

void CurrentInfo::setFrequency(const float &frequency) {
    this->frequency = frequency;
}

float CurrentInfo::getFrequency() const {
    return this->frequency;
}

void CurrentInfo::setRiskStatus(const std::string &risk_status) {
    this->risk_status = risk_status;
}

std::string CurrentInfo::getRiskStatus() const {
    return this->risk_status;
}