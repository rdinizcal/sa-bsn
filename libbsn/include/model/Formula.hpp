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
                ~Formula();

                Formula(const Formula &);
                Formula &operator=(const Formula &);

                double apply(const std::vector<std::string> parameters, const std::vector<double> values);
                Lepton::CompiledExpression getExpression() const;
                void setExpression(const Lepton::CompiledExpression &);

            private:
                Lepton::CompiledExpression expression;
        };
    }
}

#endif