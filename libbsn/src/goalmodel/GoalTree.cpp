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

        std::map<std::string, Node*> GoalTree::getNodes() const {
            return this->nodes;
        }

        void GoalTree::addRootGoal(Goal &rootgoal) {
            if (nodes.size() >= 1) throw std::invalid_argument("No more than 1 root goals allowed");

            this->nodes.insert(std::pair<std::string, Goal*> (rootgoal.getID(), &rootgoal));
        }

        void GoalTree::addNode(Node &node, const std::string &parent_id){ 
            Node *parent = this->getNode(parent_id); 

            /* if parent doesnt find child, it throws an out_of_range excp and we must add it,
                otherwise, it already has that child and we simply add it to the tree*/
            try {
                parent->getChild(node.getID());
            } catch (std::out_of_range & err) {
                parent->addChild(node);
            }

            this->nodes.insert(std::pair<std::string, Node*> (node.getID(), &node));

            if (node.hasChildren()){
                std::vector<Node> children = node.getChildren();

                for (std::vector<Node>::iterator it = children.begin(); it != children.end(); ++it ){
                    this->addNode((*it), node.getID()); 
                } 
            }
        }

        Node* GoalTree::getNode(const std::string &nodeID) {
            auto it = this->getNodes().find(nodeID);
            if (it == this->getNodes().end()) throw std::out_of_range("Could not find node");

            return this->getNodes().at(nodeID);
        }

        int GoalTree::getSize() const {
            return this->getNodes().size();
        }
    }
}