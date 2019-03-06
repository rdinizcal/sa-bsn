#include "goalmodel/Task.hpp"

namespace bsn {
    namespace goalmodel {
        
        Task::Task(const std::string &id, const std::string &description) : Node(id, description) {}
        Task::Task() : Node() {}
        Task::~Task(){}

        void Task::addChild(const Task &task) {
            this->children.push_back(task);
        }

        void Task::addChild(const Goal &goal) {
            throw std::invalid_argument("Tasks cannot contain goals as children");
        }
    }
}