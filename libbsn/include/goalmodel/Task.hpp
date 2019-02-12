#ifndef TASK_HPP
#define TASK_HPP

#include <string>

namespace bsn {
    namespace goalmodel {

        class Task {

            public:
                Task(const std::string &/*task_id*/, const std::string &/*description*/, const std::string &/*cost_symbol*/, const std::string &/*reliability_symbol*/, const std::string &/*frequency_symbol*/);
                Task(const std::string &/*task_id*/, const std::string &/*description*/, const std::string &/*cost_symbol*/, const double &/*cost*/, const std::string &/*reliability_symbol*/, const double &/*reliability*/, const std::string &/*frequency_symbol*/, const double &/*frequency*/);
                
                Task();
                ~Task();

                Task(const Task &);
                Task &operator=(const Task &);

                void setTask(const std::string &/*task_id*/);
                std::string getTask() const;

                void setDescription(const std::string &/*description*/);
                std::string getDescription() const;

                void setCostSymbol(const std::string &/*cost_symbol*/);
                std::string getCostSymbol() const;

                void setCost(const double &/*cost*/);
                double getCost() const;

                void setReliabilitySymbol(const std::string &/*reliability_symbol*/);
                std::string getReliabilitySymbol() const;

                void setReliability(const double &/*reliability*/);
                double getReliability() const;

                void setFrequencySymbol(const std::string &/*frequency_symbol*/);
                std::string getFrequencySymbol() const;

                void setFrequency(const double &/*frequency*/);
                double getFrequency() const;

            private:
                std::string task_id;
                std::string description;

                std::string cost_symbol;
                double cost;

                std::string reliability_symbol;
                double reliability;

                std::string frequency_symbol;
                double frequency;
        };
    }  
}

#endif