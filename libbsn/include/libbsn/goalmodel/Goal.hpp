#ifndef GOALMODEL_GOAL_HPP
#define GOALMODEL_GOAL_HPP

#include <string>
#include <vector>
#include <stdexcept> 

#include "libbsn/goalmodel/Node.hpp"

namespace bsn {
    namespace goalmodel {

        class Goal : public Node {

            public:
                Goal(const std::string &/*id*/, const std::string &/*description*/);
                Goal();
                ~Goal();
        };
    }  
}

#endif