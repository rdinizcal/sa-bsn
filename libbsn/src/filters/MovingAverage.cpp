#include "bsn/filters/MovingAverage.hpp"
#include <iostream>

using namespace std;

namespace bsn {
    namespace filters{
        
        MovingAverage::MovingAverage() : computedAverage(), lastInserted(), range(), buffer() {}

        MovingAverage::MovingAverage(int32_t max) : computedAverage(0.0), lastInserted(0.0), range(max), buffer({}) {}

        MovingAverage::MovingAverage(const MovingAverage &obj) :
            computedAverage(0),
            lastInserted(0),
            range(obj.getRange()),
            buffer({}) {}
        
        MovingAverage& MovingAverage::operator=(const MovingAverage &obj) {
            range = obj.getRange();          
            return (*this);
        }

        int32_t defineType(std::string type) {
            if(type == "thermometer")
                return 0;
            if(type == "ecg")
                return 1;
            if(type == "oximeter")
                return 2;
            if(type == "bpms") 
                return 3;
            if(type == "bpmd")
                return 4;
            return 0;
        }

        double MovingAverage::getValue(std::string type) {
            int32_t index = defineType(type);
            computedAverage = 0;

            if(buffer[index].size() < range){
                return buffer[index].back();
            }
            else {
                if(buffer[index].size() == range) {
                    for(double i : buffer[index]) {
                        computedAverage += i;
                    }
                    computedAverage /= range;
                }
                return computedAverage;

            }

        }

        void MovingAverage::insert(double value, std::string type) {
            int32_t index = defineType(type);
            lastInserted = value;   
            buffer[index].push_back(lastInserted);

            if(buffer[index].size() > range) {
                buffer[index].pop_front();
            }

        }

        uint32_t MovingAverage::getRange() const {
            return range;
        }

        void MovingAverage::setRange(const uint32_t r) {
            range = r;
        }

        const string MovingAverage::toString() const {
            stringstream sstr;

            sstr << "Computed average:" << computedAverage << "" << endl;

            return sstr.str();
        }

    }
}