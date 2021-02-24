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

                std::vector<std::string> getTerms() const;
                void setTerms(const std::vector<std::string> &);

                std::vector<double> getValues() const;
                void setValues(const std::vector<double> &);

                double evaluate();
                double apply(const std::vector<std::string> terms, const std::vector<double> values);

            private:
                Lepton::CompiledExpression expression;
                std::vector<std::string> terms;
                std::vector<double> values;

        };
    }
}

#endif