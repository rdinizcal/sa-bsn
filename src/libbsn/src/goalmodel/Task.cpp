#include <memory>
#include "libbsn/goalmodel/Task.hpp"

namespace bsn {
    namespace goalmodel {
        
        Task::Task(const std::string &id, const std::string &description) : Node(id, description) {}
        Task::Task() : Node() {}
        Task::~Task(){}

        void Task::addChild(std::shared_ptr<Task> task) {
            this->children.push_back(task);
        }

        void Task::addChild(std::shared_ptr<Goal> goal) {
            throw std::invalid_argument("Tasks cannot contain goals as children");
        }
    }
}