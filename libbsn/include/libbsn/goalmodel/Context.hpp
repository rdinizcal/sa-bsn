#ifndef CONTEXT_HPP
#define CONTEXT_HPP

#include <string>

namespace bsn {
    namespace goalmodel {

        class Context {

            public:
                Context(const std::string &/*id*/, const std::string &/*description*/, const bool &/*value*/);
                
                Context();
                ~Context();

                Context(const Context &);
                Context &operator=(const Context &);
                bool operator==(const Context &/*rhs*/);

                void setID(const std::string &/*id*/);
                std::string getID() const;

                void setDescription(const std::string &/*description*/);
                std::string getDescription() const;

                void setValue(const bool &/*value*/);
                bool getValue() const;

            private:
                std::string id;
                std::string description;
                bool value;
        };
    }  
}

#endif