#ifndef ENERGY_STATUS_MESSAGE_HPP
#define ENERGY_STATUS_MESSAGE_HPP

#include <string>

class EnergyStatusMessage {

  	public:
		EnergyStatusMessage(const std::string &name, const int64_t &timestamp, const int64_t &logical_clock, const std::string &source, const std::string &target, const std::string &cost) : name(name), timestamp(timestamp), logical_clock(logical_clock), source(source), target(target), cost(cost){};
        
        std::string getName() const { return this->name;};
        int64_t getTimestamp() const {return this->timestamp;};
        int64_t getLogicalClock() const {return this->logical_clock;};
        std::string getSource() const {return this->source;};
        std::string getTarget() const {return this->target;};
        std::string getCost() const {return this->cost;};

	private:
        std::string name;
        int64_t timestamp;
        int64_t logical_clock;
        std::string source;
        std::string target;
        std::string cost;
};

#endif 