#include "model/Formula.hpp"

namespace bsn {
    namespace model {
        
        Formula::Formula(): expression() {};
        Formula::Formula(const std::string& text) :expression() {
            expression = Lepton::Parser::parse(text).createCompiledExpression();
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

        double Formula::apply(const std::vector<std::string> parameters, const std::vector<double> values) {
            if (parameters.size() != values.size()) throw std::length_error("ERROR: Parameters and values size do not correspond to each other.");

            std::vector<std::string>::const_iterator p_iter;
            std::vector<double>::const_iterator v_iter;
            for( p_iter = parameters.begin(), v_iter = values.begin();
                    p_iter != parameters.end() && v_iter != values.end(); 
                    ++p_iter, ++v_iter ) {
                expression.getVariableReference(*p_iter) = *v_iter;
            }

            return expression.evaluate();
        }
    }
}
