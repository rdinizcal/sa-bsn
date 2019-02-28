#include "goalmodel/Goal.hpp"

namespace bsn {
    namespace goalmodel {
        
        Goal::Goal(const std::string &id, const std::string &description) : 
            id(id), 
            description(description),
            children() {}

        Goal::Goal() : id(), description(), children() {}
        
        Goal::Goal(const Goal &obj) : 
            id(obj.getID()),
            description(obj.getDescription()),
            children(obj.getChildren()) {}

        Goal& Goal::operator=(const Goal &obj) {
            id = obj.getID();  
            description = obj.getDescription(); 
            children = obj.getChildren();
            return (*this);
        }

        bool Goal::operator==(const Goal &rhs) {
            return this->id == rhs.id;
        }

        Goal::~Goal(){};

        void Goal::setID(const std::string &id) {
            this->id = id;
        }

        std::string Goal::getID() const {
            return this->id;
        }

        void Goal::setDescription(const std::string &description) {
            this->description = description;
        }

        std::string Goal::getDescription() const {
            return this->description;
        }

        std::vector<Goal> Goal::getChildren() const {
            return this->children;
        }

        void Goal::addChild(const Goal &task) {
            this->children.push_back(task);
        }

        void Goal::removeChild(const std::string &id) {
            int pos = findChild(id);
            this->children.erase(this->children.begin()+pos);
        }

        Goal Goal::getChild(const std::string &id) {
            int pos = findChild(id); 
            return this->children.at(pos);
        }

        int Goal::findChild(const std::string &id) {
            for(std::vector<Goal>::const_iterator it = this->children.begin();
                    it != this->children.end(); ++it) {
                
                if ((*it).getID()==id) return it-this->children.begin();
            }

            throw std::out_of_range("Not Found");
        }
    }
}