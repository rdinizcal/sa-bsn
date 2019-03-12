#include "goalmodel/Node.hpp"

namespace bsn {
    namespace goalmodel {
        
        Node::Node(const std::string &id, const std::string &description) : 
            id(id), 
            description(description),
            children() {}

        Node::Node() : id(), description(), children() {}
        
        Node::Node(const Node &obj) : 
            id(obj.getID()),
            description(obj.getDescription()),
            children(obj.getChildren()) {}

        Node& Node::operator=(const Node &obj) {
            id = obj.getID();  
            description = obj.getDescription(); 
            children = obj.getChildren();
            return (*this);
        }

        bool Node::operator==(const Node &rhs) {
            return this->id == rhs.id;
        }

        Node::~Node(){};

        void Node::setID(const std::string &id) {
            this->id = id;
        }

        std::string Node::getID() const {
            return this->id;
        }

        void Node::setDescription(const std::string &description) {
            this->description = description;
        }

        std::string Node::getDescription() const {
            return this->description;
        }

        bool Node::hasChildren() const {
            return this->children.size()>0;
        }

        std::vector<Node> Node::getChildren() const {
            return this->children;
        }

        void Node::addChild(const Node &node) {
            this->children.push_back(node);
        }

        void Node::removeChild(const std::string &id) {
            int pos = findChild(id);
            this->children.erase(this->children.begin()+pos);
        }

        Node Node::getChild(const std::string &id) {
            int pos = findChild(id); 
            return this->children.at(pos);
        }

        int Node::findChild(const std::string &id) {
            for(std::vector<Node>::const_iterator it = this->children.begin();
                    it != this->children.end(); ++it) {
                
                if ((*it).getID()==id) return it-this->children.begin();
            }

            throw std::out_of_range("Child Not Found");
        }
    }
}