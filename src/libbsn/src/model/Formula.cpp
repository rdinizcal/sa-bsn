#include "libbsn/model/Formula.hpp"

namespace bsn {
    namespace model {
        
        Formula::Formula(): expression(), terms(), values() {};
        Formula::Formula(const std::string& text) : expression(), terms(), values() {
            expression = Lepton::Parser::parse(text).createCompiledExpression();
        }
        Formula::Formula(const std::string& text, const std::vector<std::string> _terms, const std::vector<double> _values) : expression(), terms(), values() {
            if (_terms.size() != _values.size()) {
                throw std::length_error("ERROR: There can't be more terms than values.");
            }

            expression = Lepton::Parser::parse(text).createCompiledExpression();
            terms = _terms;
            values = _values;
        }

        Formula::~Formula() {};

        Formula& Formula::operator=(const Formula &obj) {
            expression = obj.getExpression();
            return (*this);
        }

        Lepton::CompiledExpression Formula::getExpression() const {
            return this->expression;    
        }

        void Formula::setExpression(const Lepton::CompiledExpression &newExpression) {
            this->expression = newExpression;
        }

        double Formula::evaluate(){
            return apply(terms,values);
        }

        double Formula::apply(const std::vector<std::string> terms, const std::vector<double> values) {
            if (terms.size() != values.size()) throw std::length_error("ERROR: terms and values size do not correspond to each other.");

            std::vector<std::string>::const_iterator p_iter;
            std::vector<double>::const_iterator v_iter;
            for( p_iter = terms.begin(), v_iter = values.begin();
                    p_iter != terms.end() && v_iter != values.end(); 
                    ++p_iter, ++v_iter ) {
                expression.getVariableReference(*p_iter) = *v_iter;
            }

            return expression.evaluate();
        }
    }
}
