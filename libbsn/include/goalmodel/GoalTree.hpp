#ifndef GOALMODEL_GOALTREE_HPP
#define GOALMODEL_GOALTREE_HPP

#include <string>
#include <map>
#include <stdexcept> 

#include "goalmodel/Node.hpp"
#include "goalmodel/Goal.hpp"

namespace bsn {
    namespace goalmodel {

        class GoalTree {

            public:
                GoalTree(const std::string &/*actor*/);
                GoalTree();
                ~GoalTree();

                GoalTree(const GoalTree &);
                GoalTree &operator=(const GoalTree &);
                bool operator==(const GoalTree &rhs);

                void setActor(const std::string &/*actor*/);
                std::string getActor() const;

                void addRootGoal(Goal &/*goal*/);
                void addNode (Node &/*node*/, const std::string &/*parent*/);
                Node* getNode(const std::string &/*node*/);
                
                int getSize() const;

            private:
                std::map<std::string, Node*> getNodes() const;

            private:
                std::string actor;
                std::map<std::string, Node*> nodes;
        };
    }  
}

#endif