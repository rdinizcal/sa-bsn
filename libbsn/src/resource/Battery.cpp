#include "libbsn/resource/Battery.hpp"

namespace bsn {
    namespace resource {

        using namespace std;
        
        Battery::Battery(const std::string _id, const double _capacity, const double _currentLevel, const double _unit) :
            id(_id),
            capacity(_capacity),
            currentLevel(_currentLevel),
            unit(_unit) {
            if (_capacity <= 0) {
                throw std::invalid_argument ("Capacity should not be negative or null");
            }

            if (_currentLevel < 0 || _currentLevel > _capacity ) {
                throw std::invalid_argument ("Current level should not be negative or null nor bigger than the capacity");
            }

            if (_unit < 0  || _unit > _capacity) {
                throw std::invalid_argument ("The resolution should not be negative or null nor bigger than the capacity");
            }
        }

        Battery::Battery() :
            id(),
            capacity(100),
            currentLevel(100),
            unit(1) {}
        
        Battery::Battery(const Battery &obj) : 
            id(obj.getId()),
            capacity(obj.getCapacity()),
            currentLevel(obj.getCurrentLevel()),
            unit(obj.getUnit()) {}

        Battery& Battery::operator=(const Battery &obj) {
            id = obj.getId();  
            capacity = obj.getCapacity(); 
            currentLevel = obj.getCurrentLevel();
            unit = obj.getUnit();        
            return (*this);
        }

        Battery::~Battery(){};
        
        void Battery::consume(const double mult) {
            currentLevel-= unit*mult;

            if(currentLevel < 0) currentLevel = 0;
        }

        void Battery::generate(const double mult) {
            currentLevel += unit*mult;
            
            if(currentLevel >= capacity) currentLevel = capacity;
        }

        void Battery::setId (const string &_id) {
            id = _id;
        }

        string Battery::getId() const {
            return id;
        }

        void Battery::setCapacity (const double &_cap) {
            capacity = _cap;
        }

        double Battery::getCapacity() const{
            return capacity;
        }

        void Battery::setCurrentLevel(const double &_cLevel) {
            currentLevel = _cLevel;
        }

        double Battery::getCurrentLevel() const{
            return currentLevel;
        }

        void Battery::setUnit(const double &_unit){
            unit = _unit;
        }

        double Battery::getUnit() const{
            return unit; 
        }

        const string Battery::toString() const {
            stringstream sstr;

            sstr << "Battery: " << id  << " " << capacity << " " << currentLevel << " " << unit << endl;

            return sstr.str();
        }

    }
}