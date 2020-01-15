#ifndef OPERATION_HPP
#define OPERATION_HPP

#include<vector>
#include<string>

namespace bsn {
    namespace operation {

        class Operation {
            public:
                Operation();
                ~Operation();

                const std::vector<std::string> split(const std::string&, const char&);
        };

    }
}


#endif