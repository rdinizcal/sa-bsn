#ifndef PROCESSOR_HPP
#define PROCESSOR_HPP

#include <vector>
#include <list>
#include <string>
#include <iostream>
#include <stdint.h>

namespace bsn {
    namespace processor {
    
        double data_fuse(std::vector<double>);
        int32_t get_sensor_id(std::string);
        
        double get_value(std::string);
            
    }
}


#endif