#ifndef GOALMODEL_TASK_HPP
#define GOALMODEL_TASK_HPP

#include <string>
#include <vector>
#include <stdexcept> 

#include "libbsn/goalmodel/Node.hpp"
#include <memory>
#include "libbsn/goalmodel/Goal.hpp"

namespace bsn {
    namespace goalmodel {

        class Task : public Node {

            public:
                Task(const std::string &/*id*/, const std::string &/*description*/);

                Task();
                ~Task();

                void addChild(std::shared_ptr<Task> /*task*/);
                void addChild(std::shared_ptr<Goal> /*goal*/);
                
        };
    }  
}

#endif