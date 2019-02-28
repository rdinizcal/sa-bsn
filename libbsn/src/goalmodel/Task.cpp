#include "goalmodel/Task.hpp"

namespace bsn {
    namespace goalmodel {
        
        Task::Task(const std::string &id, const std::string &description) : Node(id, description) {}
        Task::Task() : Node() {}
        Task::~Task(){}

    }
}