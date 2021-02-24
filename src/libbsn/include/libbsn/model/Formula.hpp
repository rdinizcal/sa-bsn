#ifndef FORMULA_HPP
#define FORMULA_HPP

#include <iostream>
#include <stdexcept>
#include <string>

#include "lepton/Lepton.h"

 
namespace bsn {
    namespace model {
        class Formula {
        
            public:
                Formula();
                Formula(const std::string& text);
                Formula(const std::string& text, const std::vector<std::string> terms, const std::vector<double> values);
                ~Formula();

                Formula(const Formula &);
                Formula &operator=(const Formula &);

                Lepton::CompiledExpression getExpression() const;
                void setExpression(const Lepton::CompiledExpression &);

                std::map<std::string,double> getTermValueMap() const;
                void setTermValueMap(const std::vector<std::string> &, const std::vector<double> &);
                void setTermValueMap(const std::map<std::string,double> &);

                double evaluate();
                std::vector<std::string> getTerms();

            private:
                Lepton::CompiledExpression expression;
                std::map<std::string,double> term_value;

        };
    }
}

#endif