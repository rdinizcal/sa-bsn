#ifndef CONTROL_THEORY_METRICS_MESSAGE_HPP
#define CONTROL_THEORY_METRICS_MESSAGE_HPP

#include <string>

class ControlTheoryMetricsMessage {

  	public:
        ControlTheoryMetricsMessage(const std::string &name, const int64_t &timestamp, const int64_t &logical_clock, const std::string &source, const std::string &target, const std::string &enactor_kp, const std::string &stability,const std::string &convergence_point, const std::string &settling_time, const std::string &overshoot, const std::string &steady_state_error) : name(name), timestamp(timestamp), logical_clock(logical_clock), source(source), target(target), enactor_kp(enactor_kp), stability(stability), convergence_point(convergence_point), settling_time(settling_time), overshoot(overshoot), steady_state_error(steady_state_error){};

        std::string getName() const { return this->name;};
        int64_t getTimestamp() const {return this->timestamp;};
        int64_t getLogicalClock() const {return this->logical_clock;};
        std::string getSource() const {return this->source;};
        std::string getTarget() const {return this->target;};
        std::string getEnactorKp() const {return this->enactor_kp;};
        std::string getStability() const {return this->stability;};
        std::string getConvergencePoint() const {return this->convergence_point;};
        std::string getSettlingTime() const {return this->settling_time;};
        std::string getOvershoot() const {return this->overshoot;};
        std::string getSteadyStateError() const {return this->steady_state_error;};

	private:
        std::string name;
        int64_t timestamp;
        int64_t logical_clock;
        std::string source;
        std::string target;
        std::string enactor_kp;
        std::string stability;
        std::string convergence_point;
        std::string settling_time;
        std::string overshoot;
        std::string steady_state_error;
};

#endif 