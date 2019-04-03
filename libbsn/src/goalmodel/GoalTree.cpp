#include "goalmodel/GoalTree.hpp"

#include <iostream>

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

        std::map<std::string, Node> GoalTree::getNodes() const {
            return this->nodes;
        }

        void GoalTree::addChildren(const std::vector<Node> &children) { 
            /*
                What if the goal node has tasks nodes as children? 
                                    oh no
            */
            for (std::vector<Node>::const_iterator it = children.begin();
                         it != children.end(); ++it )
                this->addNode(*it); 
        }

        void GoalTree::addNode(const Node &node) { 
            this->nodes.insert(std::pair<std::string, Node> (node.getID(), node));
            if (node.hasChildren()) this->addChildren(node.getChildren());
        } 

        void GoalTree::addRootGoal(Goal &rootgoal) {
            if (nodes.size() >= 1) throw std::invalid_argument("No more than 1 root goals allowed");

            return this->addNode(rootgoal);
        }

        Node GoalTree::getNode(const std::string &nodeID) const { 
            try {
                return (*this->getNodes().find(nodeID)).second;
            } catch (std::out_of_range const &err) {
                throw std::out_of_range("Could not find node."); 
            } 
        }

        int GoalTree::getSize() const {
            return this->getNodes().size();
        }

        std::vector<Node> GoalTree::getLeafTasks() const {
            std::vector<Node> leafTasks;

            for(std::map<std::string, Node>::const_iterator it = nodes.begin();
                    it != nodes.end(); ++it){

                if(!(*it).second.hasChildren()) leafTasks.push_back((*it).second);

            }

            return leafTasks;
        }
        
    }
}