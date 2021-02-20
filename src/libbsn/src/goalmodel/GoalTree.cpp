#include "libbsn/goalmodel/GoalTree.hpp"

#include <iostream>
#include <memory>

namespace bsn {
    namespace goalmodel {
        
        GoalTree::GoalTree(const std::string &actor) : 
            actor(actor),
            nodes() {}

        GoalTree::GoalTree() : actor(), nodes() {}

        GoalTree::~GoalTree(){};
        
        GoalTree::GoalTree(const GoalTree &obj) : 
            actor(obj.getActor()),
            nodes(obj.getNodes()) {}

        GoalTree& GoalTree::operator=(const GoalTree &obj) {
            actor = obj.getActor();  
            nodes = obj.getNodes();
            return (*this); 
        }

        bool GoalTree::operator==(const GoalTree &rhs) {
            return this->actor == rhs.actor;
        }

        void GoalTree::setActor(const std::string &actor) {
            this->actor = actor;
        }

        std::string GoalTree::getActor() const {
            return this->actor;
        }

        std::map<std::string, std::shared_ptr<Node>> GoalTree::getNodes() const {
            return this->nodes;
        }

        void GoalTree::addChildren(std::vector<std::shared_ptr<Node>> children) { 
            /*
                What if the goal node has tasks nodes as children? 
                                    oh no
            */
            for (std::vector<std::shared_ptr<Node>>::const_iterator it = children.begin();
                         it != children.end(); ++it )
                this->addNode(*it); 
        }

        void GoalTree::addNode(std::shared_ptr<Node> node) { 
            this->nodes[node->getID()] = node;
            if (node->hasChildren()) this->addChildren(node->getChildren());
        } 

        void GoalTree::addRootGoal(std::shared_ptr<Goal> rootgoal) {
            if (nodes.size() >= 1) throw std::invalid_argument("No more than 1 root goals allowed");

            return this->addNode(rootgoal);
        }

        std::shared_ptr<Node> GoalTree::getNode(const std::string &nodeID) const { 
            try {
                return (*this->getNodes().find(nodeID)).second;
            } catch (std::out_of_range const &err) {
                throw std::out_of_range("Could not find node."); 
            } 
        }

        int GoalTree::getSize() const {
            return this->getNodes().size();
        }

        std::vector<std::shared_ptr<LeafTask>> GoalTree::getLeafTasks() const {
            std::shared_ptr<Node> temp;
            std::vector<std::shared_ptr<LeafTask>> leafTasks;

             for(std::map<std::string, std::shared_ptr<Node>>::const_iterator it = nodes.begin();
                     it != nodes.end(); it++)
             {
                if(!(it->second->hasChildren()))
                    leafTasks.push_back(std::dynamic_pointer_cast<LeafTask>(it->second));    
             }

            return leafTasks;
        }
        
    }
}