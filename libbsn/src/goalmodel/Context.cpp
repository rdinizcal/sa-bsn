#include "goalmodel/Context.hpp"

namespace bsn {
    namespace goalmodel {
        
        Context::Context(const std::string &_context_id, const bool &_value, const std::string &_context_symbol) :
            context_id(_context_id),
            value(_value),
            context_symbol(_context_symbol) {}

        Context::Context() :
            context_id(),
            value(),
            context_symbol() {}
        
        Context::Context(const Context &obj) : 
            context_id(obj.getContext()),
            value(obj.getValue()),
            context_symbol(obj.getContextSymbol()) {}

        Context& Context::operator=(const Context &obj) {
            context_id = obj.getContext();  
            value = obj.getValue();
            context_symbol = obj.getContextSymbol();  
            return (*this);
        }

        Context::~Context(){};
        
        void Context::setContext(const std::string &_context_id) {
            context_id = _context_id;
        }

        std::string Context::getContext() const {
            return context_id;
        }

        void Context::setValue(const bool &_value) {
            value = _value;
        }

        bool Context::getValue() const {
            return value;
        }

        void Context::setContextSymbol(const std::string &_context_symbol) {
            context_symbol = _context_symbol;
        }

        std::string Context::getContextSymbol() const {
            return context_symbol;
        }

    }
}