#include "goalmodel/Task.hpp"

namespace bsn {
    namespace goalmodel {
        
        Task::Task(const std::string &id, const std::string &description, const goalmodel::Context &context, const goalmodel::Property &cost, const  goalmodel::Property &reliability, const goalmodel::Property &frequency) : 
            id(id), 
            description(description), 
            context(context), 
            cost(cost), 
            reliability(reliability), 
            frequency(frequency) {}

        Task::Task() : id(), description(), context(), cost(), reliability(), frequency() {}
        
        Task::Task(const Task &obj) : 
            id(obj.getID()),
            description(obj.getDescription()),
            context(obj.getContext()),
            cost(obj.getCost()),
            reliability(obj.getReliability()),
            frequency(obj.getFrequency()) {}

        Task& Task::operator=(const Task &obj) {
            id = obj.getID();  
            description = obj.getDescription(); 
            context = obj.getContext();
            cost = obj.getCost();
            reliability = obj.getReliability();
            frequency = obj.getFrequency();        
            return (*this);
        }

        Task::~Task(){};

        void Task::setID(const std::string &id) {
            this->id = id;
        }

        std::string Task::getID() const {
            return id;
        }

        void Task::setDescription(const std::string &description) {
            this->description = description;
        }

        std::string Task::getDescription() const {
            return description;
        }

        void Task::setContext(const goalmodel::Context &context) {
            this->context = context;
        }

        goalmodel::Context Task::getContext() const {
            return this->context;
        }

        void Task::setCost(const goalmodel::Property &cost) {
            this->cost = cost;
        }

        goalmodel::Property Task::getCost() const {
            return cost;
        }

        void Task::setReliability(const goalmodel::Property &reliability) {
            this->reliability = reliability;
        }

        goalmodel::Property Task::getReliability() const {
            return reliability;
        }

        void Task::setFrequency(const goalmodel::Property &frequency) {
            this->frequency = frequency;
        }

        goalmodel::Property Task::getFrequency() const {
            return frequency;
        }
    }
}