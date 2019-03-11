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

        std::map<std::string, Node*> GoalTree::getNodes() const {
            return this->nodes;
        }

        void GoalTree::addRootGoal(Goal &rootgoal) {
            if (nodes.size() >= 1) throw std::invalid_argument("No more than 1 root goals allowed");

            this->nodes.insert(std::pair<std::string, Goal*> (rootgoal.getID(), &rootgoal));
        }

        void GoalTree::addNode(Node &node, const std::string &parent_id){ 
            Node *parent = this->getNode(parent_id); 
            parent->addChild(node);

            std::cout << "Parent: " << parent->getID() << ": " << parent->getDescription() << std::endl;
            std::cout << parent->getChildren().size() << " / " << parent->getChildren().at(0).getID() << std::endl;

            this->nodes.insert(std::pair<std::string, Node*> (node.getID(), &node));
        }

        Node* GoalTree::getNode(const std::string &nodeID) {
            return this->getNodes().at(nodeID);
        }

        int GoalTree::getSize() const {
            return this->getNodes().size();
        }
    }
}