#include "libbsn/goalmodel/Context.hpp"

namespace bsn {
    namespace goalmodel {
        
        Context::Context(const std::string &id, const std::string &description, const bool &value) :
            id(id),
            description(description),
            value(value) {}

        Context::Context() :
            id(),
            description(),
            value() {}

        Context::~Context(){};
        
        Context::Context(const Context &obj) : 
            id(obj.getID()),
            description(obj.getDescription()),
            value(obj.getValue()) {}

        Context& Context::operator=(const Context &obj) {
            this->id = obj.getID();  
            this->description = obj.getDescription();  
            this->value = obj.getValue();
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