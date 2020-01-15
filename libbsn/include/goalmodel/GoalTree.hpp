#ifndef GOALMODEL_GOALTREE_HPP
#define GOALMODEL_GOALTREE_HPP

#include <string>
#include <map>
#include <stdexcept> 
#include <vector>
#include <memory>

#include "goalmodel/Node.hpp"
#include "goalmodel/Goal.hpp"
#include "goalmodel/Task.hpp"
#include "goalmodel/LeafTask.hpp"
 
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

            private:
                std::map<std::string, std::shared_ptr<Node>> getNodes() const;
                void addChildren(const std::vector<std::shared_ptr<Node>> /*child*/);
                void addNode(std::shared_ptr<Node> /*node*/);
            
            public:
                void setActor(const std::string &/*actor*/);
                std::string getActor() const;

                void addRootGoal(std::shared_ptr<Goal> /*goal*/);
                std::shared_ptr<Node> getNode(const std::string &/*node*/) const;
                
                int getSize() const;

                std::vector<std::shared_ptr<LeafTask>> getLeafTasks() const;

            private:
                std::string actor;
                std::map<std::string, std::shared_ptr<Node>> nodes;
        };
    }  
}

#endif