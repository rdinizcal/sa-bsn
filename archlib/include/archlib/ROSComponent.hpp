#ifndef ROSCOMPONENT_HPP
#define ROSCOMPONENT_HPP

#include <string>
#include <stdexcept>

#include "ros/ros.h"

#include "archlib/ROSComponentDescriptor.hpp"

namespace arch {
    class ROSComponent{

        public:
            ROSComponent(int &argc, char **argv, const std::string &name);
		    virtual ~ROSComponent();

        private:
            ROSComponent(const ROSComponent &);
		    ROSComponent &operator=(const ROSComponent &);

        public:
            virtual void setUp() = 0;
            virtual void tearDown() = 0;
            virtual int32_t run();
            virtual void body() = 0;

        protected:
            ROSComponentDescriptor rosComponentDescriptor;    

            static std::string getRosNodeName(const std::string& node_name, const std::string& node_namespace);
    };
}

#endif
