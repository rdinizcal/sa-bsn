#include "libbsn/goalmodel/LeafTask.hpp"
#include <memory>

namespace bsn {
    namespace goalmodel {
        
        LeafTask::LeafTask(const std::string &id, const std::string &description, const goalmodel::Context &context, const goalmodel::Property &cost, const  goalmodel::Property &reliability, const goalmodel::Property &frequency) : 
            Task(id, description),
            context(context), 
            cost(cost), 
            reliability(reliability), 
            frequency(frequency) {}
        
        LeafTask::LeafTask(const std::string &id, const std::string &description, const goalmodel::Property &cost, const  goalmodel::Property &reliability, const goalmodel::Property &frequency) : 
            Task(id, description),
            context(),
            cost(cost), 
            reliability(reliability), 
            frequency(frequency) {}

        LeafTask::LeafTask() : Task(), context(), cost(), reliability(), frequency() {}
        LeafTask::~LeafTask(){};
        
        LeafTask::LeafTask(const LeafTask &obj) : 
            Task(obj.getID(), obj.getDescription()),
            context(obj.getContext()),
            cost(obj.getCost()),
            reliability(obj.getReliability()),
            frequency(obj.getFrequency()) {}

        LeafTask& LeafTask::operator=(const LeafTask &obj) {
            id = obj.getID();
            description = obj.getDescription();
            context = obj.getContext();
            cost = obj.getCost();
            reliability = obj.getReliability();
            frequency = obj.getFrequency();        
            return (*this);
        }

        void LeafTask::setContext(const goalmodel::Context &context) {
            this->context = context;
        }

        goalmodel::Context LeafTask::getContext() const {
            return this->context;
        }

        void LeafTask::setCost(const goalmodel::Property &cost) {
            this->cost = cost;
        }

        goalmodel::Property LeafTask::getCost() const {
            return cost;
        }

        void LeafTask::setReliability(const goalmodel::Property &reliability) {
            this->reliability = reliability;
        }

        goalmodel::Property LeafTask::getReliability() const {
            return reliability;
        }

        void LeafTask::setFrequency(const goalmodel::Property &frequency) {
            this->frequency = frequency;
        }

        goalmodel::Property LeafTask::getFrequency() const {
            return frequency;
        }

        void LeafTask::addChild(std::shared_ptr<Node> node) {
            throw std::invalid_argument("Leaf Tasks cannot contain children");
        }
    }
}