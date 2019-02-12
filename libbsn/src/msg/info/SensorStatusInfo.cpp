#include "msg/info/SensorStatusInfo.h"

namespace bsn {
    namespace msg { 
        namespace info {
        
            using namespace std;

            SensorStatusInfo::SensorStatusInfo() : 
                value(){}
            
            SensorStatusInfo::SensorStatusInfo(const double &_value) : 
                value(_value) {}
            
            SensorStatusInfo::~SensorStatusInfo() {}
            
            SensorStatusInfo::SensorStatusInfo(const SensorStatusInfo &obj) :
                SerializableData(),
                value(obj.getValue()){}
            
            SensorStatusInfo& SensorStatusInfo::operator=(const SensorStatusInfo &obj) {
                value = obj.getValue();      
                return (*this);
            }
            
            int32_t SensorStatusInfo::ID() {
                return 878;
            }
            const string SensorStatusInfo::ShortName() {
                return "SensorStatusInfo";
            }
            const string SensorStatusInfo::LongName() {
                return "info.SensorStatusInfo";
            }
            
            int32_t SensorStatusInfo::getID() const {
                return SensorStatusInfo::ID();
            }
            const string SensorStatusInfo::getShortName() const {
                return SensorStatusInfo::ShortName();
            }
            const string SensorStatusInfo::getLongName() const {
                return SensorStatusInfo::LongName();
            }

            void SensorStatusInfo::setValue(const double &_value) {
                value = _value;
            }

            double SensorStatusInfo::getValue() const{
                return value;
            }
            
            ostream& SensorStatusInfo::operator<<(ostream &out) const {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Serializer> s = sf.getQueryableNetstringsSerializer(out);
                
                s->write(1, value);

                return out;
            }
            
            istream& SensorStatusInfo::operator>>(istream &in) {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Deserializer> d = sf.getQueryableNetstringsDeserializer(in);
                
                d->read(1, value);

                return in;
            }
            
            const string SensorStatusInfo::toString() const {
                stringstream sstr;

                sstr << "SensorStatusInfo#" << endl;
                sstr << "VALUE:" << value << endl;

                return sstr.str();
            }
        }
    }
}