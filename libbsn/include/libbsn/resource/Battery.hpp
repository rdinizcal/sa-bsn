#ifndef BATTERY_HPP
#define BATTERY_HPP

#include <sstream>
#include <string>
#include <stdint.h>
#include <stdexcept>

namespace bsn {
    namespace resource {

        class Battery {

            public:
                Battery(const std::string /*id*/, const double /*capacity*/, const double /*currentLevel*/, const double /*unit*/);
                
                Battery();
                ~Battery();

                Battery(const Battery & /*obj*/);
                Battery &operator=(const Battery & /*obj*/);

                void consume(const double /*mult*/);
                void generate(const double /*mult*/);

                void setId(const std::string &/*id*/);
                std::string getId() const;

                void setCapacity(const double &/*capacity*/);
                double getCapacity() const; 

                void setCurrentLevel (const double &/*currentLevel*/);
                double getCurrentLevel() const;

                void setUnit(const double &/*unit*/);
                double getUnit() const;

                const std::string toString() const;

            private:
                std::string id;
                double capacity;
                double currentLevel;
                double unit;
        };
    }  
}

#endif