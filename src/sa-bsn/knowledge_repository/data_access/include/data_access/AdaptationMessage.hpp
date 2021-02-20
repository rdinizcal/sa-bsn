#ifndef ADAPTATION_MESSAGE_HPP
#define ADAPTATION_MESSAGE_HPP

#include <string>

class AdaptationMessage {

  	public:
        AdaptationMessage(const std::string &name, const int64_t &timestamp, const int64_t &logical_clock, const std::string &source, const std::string &target, const std::string &content) : name(name), timestamp(timestamp), logical_clock(logical_clock), source(source), target(target), content(content){};

        std::string getName() const { return this->name;};
        int64_t getTimestamp() const {return this->timestamp;};
        int64_t getLogicalClock() const {return this->logical_clock;};
        std::string getSource() const {return this->source;};
        std::string getTarget() const {return this->target;};
        std::string getContent() const {return this->content;};

	private:
        std::string name;
        int64_t timestamp;
        int64_t logical_clock;
        std::string source;
        std::string target;
        std::string content;
};

#endif 