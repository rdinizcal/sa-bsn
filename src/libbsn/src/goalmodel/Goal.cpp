#include "libbsn/goalmodel/Goal.hpp"

namespace bsn {
    namespace goalmodel {
        
        Goal::Goal(const std::string &id, const std::string &description) : Node(id, description) {}
        Goal::Goal() : Node() {}
        Goal::~Goal(){}

    }
}