#include "libbsn/goalmodel/Node.hpp"

#include <iostream>
#include <memory>

namespace bsn {
    namespace goalmodel {
        
        Node::Node() : id(), description() {}

        Node::~Node(){};

        Node::Node(const std::string &id, const std::string &description) : id(id), description(description), children() {}
        
        Node::Node(const Node &obj) : id(obj.getID()), description(obj.getDescription()), children(obj.getChildren()){}

        Node& Node::operator=(const Node &obj) {
            if(this != &obj){
                this->id = obj.getID();  
                this->description = obj.getDescription(); 
                this->children = obj.getChildren();
            }

            return *this;
        }

        bool Node::operator==(const Node &rhs) const {
            return  this->id == rhs.getID() && 
                    this->description == rhs.getDescription();
        }

        std::ostream& operator<<(std::ostream &stream, const Node &node) {
            return stream << "bsn::goalmodel::Node(" << node.getID() << ", " << node.getDescription() << ", " "has " << node.getChildren().size() << " children" << ")";
        }

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
            return !this->children.empty();
        }

        std::vector<std::shared_ptr<Node>> Node::getChildren() const {
            return this->children;
        }

        void Node::addChild(std::shared_ptr<Node> node) {
            this->children.push_back(node);
        }

        void Node::removeChild(const std::string &id) {
            int pos = findChild(id);
            this->children.erase(this->children.begin()+pos);
        }

        std::shared_ptr<Node> Node::getChild(const std::string &id) {
            int pos = findChild(id); 
            return children[pos];
        }

        int Node::findChild(const std::string &id) {
            for(std::vector<std::shared_ptr<Node>>::const_iterator it = this->children.begin();
                    it != this->children.end(); ++it) {
                
                if ((*it)->getID()==id) return it-this->children.begin();
            }

            throw std::out_of_range("Child Not Found");
        }
    }
}