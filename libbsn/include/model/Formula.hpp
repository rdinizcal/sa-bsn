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
                Formula(const std::string& text);
                ~Formula();

                double apply(const std::vector<std::string> parameters, const std::vector<double> values);

            private:
                Lepton::CompiledExpression expression;
        };
    }
}

#endif