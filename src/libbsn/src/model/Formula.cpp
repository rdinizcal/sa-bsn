#include "libbsn/model/Formula.hpp"

namespace bsn {
    namespace model {
        
        Formula::Formula(): expression(), parameters(), values() {};
        Formula::Formula(const std::string& text) : expression(), parameters(), values() {
            expression = Lepton::Parser::parse(text).createCompiledExpression();
        }
        Formula::Formula(const std::string& text, const std::vector<std::string> _parameters, const std::vector<double> _values) : expression(), parameters(), values() {
            if (_parameters.size() != _values.size()) {
                throw std::length_error("ERROR: There can't be more parameters than values.");
            }

            expression = Lepton::Parser::parse(text).createCompiledExpression();
            parameters = _parameters;
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
            return apply(parameters,values);
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
