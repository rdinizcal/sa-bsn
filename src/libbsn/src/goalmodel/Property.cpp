#include "libbsn/goalmodel/Property.hpp"

namespace bsn {
    namespace goalmodel {
        
        Property::Property(const std::string &id, const double &value) : id(id), value(value) {}

        Property::Property() : id(), value() {}
        
        Property::~Property(){};

        Property::Property(const Property &obj) : 
            id(obj.getID()),
            value(obj.getValue()) {}

        Property& Property::operator=(const Property &obj) {
            id = obj.getID();  
            value = obj.getValue(); 
            return (*this);
        } 

        bool Property::operator==(const Property &rhs) {
            return this->id == rhs.id;
        }

        void Property::setID(const std::string &id) {
            this->id = id;
        }

        std::string Property::getID() const {
            return id;
        }

        void Property::setValue(const double &value) {
            this->value = value;
        }

        double Property::getValue() const {
            return value;
        }

    }
}