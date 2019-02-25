#include "goalmodel/Task.hpp"

namespace bsn {
    namespace goalmodel {
        
        Task::Task(const std::string &id, const std::string &description) : 
            id(id), 
            description(description),
            children() {}

        Task::Task() : id(), description(), children() {}
        
        Task::Task(const Task &obj) : 
            id(obj.getID()),
            description(obj.getDescription()),
            children(obj.getChildren()) {}

        Task& Task::operator=(const Task &obj) {
            id = obj.getID();  
            description = obj.getDescription(); 
            children = obj.getChildren();
            return (*this);
        }

        bool Task::operator==(const Task &rhs) {
            return this->id == rhs.id;
        }

        Task::~Task(){};

        void Task::setID(const std::string &id) {
            this->id = id;
        }

        std::string Task::getID() const {
            return this->id;
        }

        void Task::setDescription(const std::string &description) {
            this->description = description;
        }

        std::string Task::getDescription() const {
            return this->description;
        }

        std::vector<Task> Task::getChildren() const {
            return this->children;
        }

        void Task::addChild(const Task &task) {
            this->children.push_back(task);
        }

        void Task::removeChild(const std::string &id) {
            int pos = findChild(id);
            this->children.erase(this->children.begin()+pos);
        }

        Task Task::getChild(const std::string &id) {
            int pos = findChild(id); 
            return this->children.at(pos);
        }

        //TODO: throw exception when doesnt find child
        int Task::findChild(const std::string &id) {
            for(std::vector<Task>::const_iterator it = this->children.begin();
                    it != this->children.end(); ++it) {
                
                if ((*it).getID()==id) return it-this->children.begin();
            }

            throw std::out_of_range("Not Found");
        }
    }
}