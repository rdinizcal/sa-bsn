#include "goalmodel/Task.hpp"

namespace bsn {
    namespace goalmodel {
        
        Task::Task(const std::string &_task_id, const std::string &_description, const std::string &_cost_symbol, const std::string &_reliability_symbol, const std::string &_frequency_symbol) : task_id(_task_id), description(_description), cost_symbol(_cost_symbol), cost(0), reliability_symbol(_reliability_symbol), reliability(0), frequency_symbol(_frequency_symbol), frequency(0) {}

        Task::Task(const std::string &_task_id, const std::string &_description, const std::string &_cost_symbol, const double &_cost, const std::string &_reliability_symbol, const double &_reliability, const std::string &_frequency_symbol, const double &_frequency) : task_id(_task_id), description(_description), cost_symbol(_cost_symbol), cost(_cost), reliability_symbol(_reliability_symbol), reliability(_reliability), frequency_symbol(_frequency_symbol), frequency(_frequency){}

        Task::Task() : task_id(), description(), cost_symbol(), cost(), reliability_symbol(), reliability(), frequency_symbol(), frequency() {}
        
        Task::Task(const Task &obj) : 
            task_id(obj.getTask()),
            description(obj.getDescription()),
            cost_symbol(obj.getCostSymbol()),
            cost(obj.getCost()),
            reliability_symbol(obj.getReliabilitySymbol()),
            reliability(obj.getReliability()),
            frequency_symbol(obj.getFrequencySymbol()),
            frequency(obj.getFrequency()) {}

        Task& Task::operator=(const Task &obj) {
            task_id = obj.getTask();  
            description = obj.getDescription(); 
            cost_symbol = obj.getCostSymbol();
            cost = obj.getCost();
            reliability_symbol = obj.getReliabilitySymbol();        
            reliability = obj.getReliability();
            frequency_symbol = obj.getFrequencySymbol();        
            frequency = obj.getFrequency();        
            return (*this);
        }

        Task::~Task(){};

        void Task::setTask(const std::string &_task_id) {
            task_id = _task_id;
        }

        std::string Task::getTask() const {
            return task_id;
        }

        void Task::setDescription(const std::string &_description) {
            description = _description;
        }

        std::string Task::getDescription() const {
            return description;
        }

        void Task::setCostSymbol(const std::string &_cost_symbol) {
            cost_symbol = _cost_symbol;
        }

        std::string Task::getCostSymbol() const {
            return cost_symbol;
        }

        void Task::setCost(const double &_cost) {
            cost = _cost;
        }

        double Task::getCost() const {
            return cost;
        }

        void Task::setReliabilitySymbol(const std::string &_reliability_symbol) {
            reliability_symbol = _reliability_symbol;
        }

        std::string Task::getReliabilitySymbol() const{
            return reliability_symbol;
        }

        void Task::setReliability(const double &_reliability) {
            reliability = _reliability;
        }

        double Task::getReliability() const {
            return reliability;
        }

        void Task::setFrequencySymbol(const std::string &_frequency_symbol) {
            frequency_symbol = _frequency_symbol;
        }

        std::string Task::getFrequencySymbol() const{
            return frequency_symbol;
        }

        void Task::setFrequency(const double &_frequency) {
            frequency = _frequency;
        }

        double Task::getFrequency() const {
            return frequency;
        }
    }
}