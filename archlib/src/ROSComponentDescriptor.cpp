#include "archlib/ROSComponentDescriptor.hpp"

namespace arch {

    ROSComponentDescriptor::ROSComponentDescriptor() : name(), freq(1) {}
    ROSComponentDescriptor::~ROSComponentDescriptor() {}

    ROSComponentDescriptor::ROSComponentDescriptor(const ROSComponentDescriptor &obj) : 
        name(obj.getName()), 
        freq(obj.getFreq()) {}

    ROSComponentDescriptor& ROSComponentDescriptor::operator=(const ROSComponentDescriptor &obj) {
        this->name = obj.getName();
        this->freq = obj.getFreq();
    }

    void ROSComponentDescriptor::setName(const std::string &name){
        this->name = name;
    }

    std::string ROSComponentDescriptor::getName() const {
        return this->name;
    }

    void ROSComponentDescriptor::setFreq(const double &freq) {
        this->freq = freq;
    }

    double ROSComponentDescriptor::getFreq() const {
        return this->freq;
    }
}