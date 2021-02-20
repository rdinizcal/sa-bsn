#include "libbsn/filters/MovingAverage.hpp"
#include <iostream>

using namespace std;

namespace bsn {
    namespace filters{
        
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

        double MovingAverage::getValue() {
     
            computedAverage = 0.0;

            // Se a lista ainda não foi preenchida
            if(buffer.size() < range && buffer.size() > 0){
                for(double i : buffer) {
                    computedAverage += i;
                    }
                computedAverage /= buffer.size();
                return computedAverage;
            }
            // Quando a lista possui o limite máximo de valores
            else if(buffer.size() == range) {
                for(double i : buffer) {
                    computedAverage += i;
                    }
                computedAverage /= range;
                return computedAverage;
            }
			// Qualquer outro range de valores
            return 0;

        }

        void MovingAverage::insert(double value) {            
            lastInserted = value;   
            buffer.push_back(lastInserted);

            if(buffer.size() > range) {
                buffer.pop_front();
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

        std::list<double> MovingAverage::getBuffer(){

            return buffer;
        }

    }
}