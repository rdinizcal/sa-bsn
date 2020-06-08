#ifndef ENGINE_INFO_MESSAGE_HPP
#define ENGINE_INFO_MESSAGE_HPP

#include <string>

class EngineInfoMessage {

  	public:
        EngineInfoMessage(const std::string &name, const int64_t &timestamp, const int64_t &logical_clock, const std::string &source, const std::string &target, const std::string &engine_kp, const std::string &engine_offset, const std::string &elapsed_time) : name(name), timestamp(timestamp), logical_clock(logical_clock), source(source), target(target), engine_kp(engine_kp), engine_offset(engine_offset), elapsed_time(elapsed_time){};

        std::string getName() const { return this->name;};
        int64_t getTimestamp() const {return this->timestamp;};
        int64_t getLogicalClock() const {return this->logical_clock;};
        std::string getSource() const {return this->source;};
        std::string getTarget() const {return this->target;};
        std::string getEngineKp() const {return this->engine_kp;};
        std::string getEngineOffset() const {return this->engine_offset;};
        std::string getElapsedTime() const {return this->elapsed_time;};

	private:
        std::string name;
        int64_t timestamp;
        int64_t logical_clock;
        std::string source;
        std::string target;
        std::string engine_kp;
        std::string engine_offset;
        std::string elapsed_time;
};

#endif 