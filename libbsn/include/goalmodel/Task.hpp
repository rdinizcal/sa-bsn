#ifndef GOALMODEL_TASK_HPP
#define GOALMODEL_TASK_HPP

#include <string>
#include <vector>
#include <stdexcept> 

#include "goalmodel/Node.hpp"

namespace bsn {
    namespace goalmodel {

        class Task : public Node {

            public:
                Task(const std::string &/*id*/, const std::string &/*description*/);

                Task();
                ~Task();
        };
    }  
}

#endif