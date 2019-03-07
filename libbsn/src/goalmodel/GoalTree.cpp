#include "goalmodel/GoalTree.hpp"

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

        void GoalTree::addRootGoal(const Goal &rootgoal) {
            if (nodes.size() >= 1) throw std::invalid_argument("No more than 1 root goals allowed");

            nodes.insert(std::pair<std::string, Goal> (rootgoal.getID(), rootgoal));
        }

        Node GoalTree::getNode(const std::string &nodeID) {
            return getNodes()[nodeID];
        }

        int GoalTree::getSize() {
            return getNodes().size();
        }
    }
}