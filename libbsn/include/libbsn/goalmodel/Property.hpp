#ifndef GOALMODEL_PROPERTY_HPP
#define GOALMODEL_PROPERTY_HPP

#include <string>

namespace bsn {
    namespace goalmodel {

        class Property {

            public:
                Property(const std::string &/*id*/, const double &/*value*/);

                Property();
                ~Property();

                Property(const Property &);
                Property &operator=(const Property &);
                bool operator==(const Property &/*rhs*/);

                void setID(const std::string &/*id*/);
                std::string getID() const;

                void setValue(const double &/*value*/);
                double getValue() const;

            private:
                std::string id;
                double value;

        };
    }  
}

#endif