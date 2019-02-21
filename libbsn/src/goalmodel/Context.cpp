#include "goalmodel/Context.hpp"

namespace bsn {
    namespace goalmodel {
        
        Context::Context(const std::string &id, const std::string &description, const bool &value) :
            id(id),
            value(value),
            description(description) {}

        Context::Context() :
            id(),
            value(),
            description() {}

        Context::~Context(){};
        
        Context::Context(const Context &obj) : 
            id(obj.getID()),
            value(obj.getValue()),
            description(obj.getDescription()) {}

        Context& Context::operator=(const Context &obj) {
            this->id = obj.getID();  
            this->value = obj.getValue();
            this->description = obj.getDescription();  
            return (*this);
        }

        bool Context::operator==(const Context &rhs) {
            return this->id == rhs.id;
        }
        
        void Context::setID(const std::string &id) {
            this->id = id;
        }

        std::string Context::getID() const {
            return this->id;
        }

        void Context::setDescription(const std::string &description) {
            this->description = description;
        }

        std::string Context::getDescription() const {
            return this->description;
        }

        void Context::setValue(const bool &value) {
            this->value = value;
        }

        bool Context::getValue() const {
            return this->value;
        }


    }
}