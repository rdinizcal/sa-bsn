#include "libbsn/model/Formula.hpp"

namespace bsn {
    namespace model {
        
        Formula::Formula(): expression(), term_value() {};
        Formula::Formula(const std::string& text) : expression(), term_value() {
            expression = Lepton::Parser::parse(text).createCompiledExpression();
        }
        Formula::Formula(const std::string& text, const std::vector<std::string> terms, const std::vector<double> values) : expression(), term_value() {
            if (terms.size() != values.size()) {
                throw std::length_error("ERROR: terms and values size do not correspond to each other.");
            }

            expression = Lepton::Parser::parse(text).createCompiledExpression();
            for (size_t i = 0; i < terms.size(); i++){
                term_value[terms.at(i)] = values.at(i);
            }
        }

        Formula::~Formula() {};

        Formula::Formula(const Formula &obj) : expression(obj.getExpression()), term_value(obj.getTermValueMap()) {}

        Formula& Formula::operator=(const Formula &obj) {
            expression = obj.getExpression();
            term_value = obj.getTermValueMap();
            return (*this);
        }

        Lepton::CompiledExpression Formula::getExpression() const {
            return this->expression;    
        }

        void Formula::setExpression(const Lepton::CompiledExpression &newExpression) {
            this->expression = newExpression;
        }

        std::map<std::string,double> Formula::getTermValueMap() const {
            return this-> term_value;
        }

        void Formula::setTermValueMap(const std::vector<std::string> &terms, const std::vector<double> &values) {
            std::vector<std::string>::const_iterator t_iter;
            std::vector<double>::const_iterator v_iter;
            for( t_iter = terms.begin(), v_iter = values.begin();
                    t_iter != terms.end() && v_iter != values.end(); 
                    ++t_iter, ++v_iter ) {
                    term_value[*t_iter] = *v_iter;
            }
        }

        void Formula::setTermValueMap(const std::map<std::string,double> &_term_value){
            term_value = _term_value;
        }

        double Formula::evaluate(){
            std::vector<std::string> keys;
            std::vector<double> values;
            for(std::map<std::string,double>::iterator it = term_value.begin(); it != term_value.end(); ++it) {
                expression.getVariableReference(it->first) = it->second;
            }

            return expression.evaluate();
        }

        /**
         @return All terms of the formula.
        */
        std::vector<std::string> Formula::getTerms() {

            std::set<std::string> terms = expression.getVariables();
            std::vector<std::string> vec_terms(terms.begin(),terms.end());
            return vec_terms;
        }
    }
}
