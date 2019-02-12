#ifndef CONTEXT_HPP
#define CONTEXT_HPP

#include <string>

namespace bsn {
    namespace goalmodel {

        class Context {

            public:
                Context(const std::string &/*context_id*/, const bool &/*value*/, const std::string &/*context_symbol*/);
                
                Context();
                ~Context();

                Context(const Context &);
                Context &operator=(const Context &);

                void setContext(const std::string &/*context_id*/);
                std::string getContext() const;

                void setValue(const bool &/*value*/);
                bool getValue() const;

                void setContextSymbol(const std::string &/*context_symbol*/);
                std::string getContextSymbol() const;

            private:
                std::string context_id;
                bool value;

                std::string context_symbol;
        };
    }  
}

#endif