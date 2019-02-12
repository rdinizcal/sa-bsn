#include "msg/data/ResourceUpdate.h"

namespace bsn {
    namespace msg { 
        namespace data {
        
            using namespace std;

            ResourceUpdate::ResourceUpdate() : units() {}
            
            ResourceUpdate::ResourceUpdate(const int32_t &_units) : 
                units(_units) {}
            
            ResourceUpdate::~ResourceUpdate() {}
            
            ResourceUpdate::ResourceUpdate(const ResourceUpdate &obj) :
                SerializableData(),
                units(obj.getUnits()){}
            
            ResourceUpdate& ResourceUpdate::operator=(const ResourceUpdate &obj) {
                units = obj.getUnits();        
                return (*this);
            }
            
            int32_t ResourceUpdate::ID() {
                return 874;
            }
            const string ResourceUpdate::ShortName() {
                return "ResourceUpdate";
            }
            const string ResourceUpdate::LongName() {
                return "data.ResourceUpdate";
            }
            
            int32_t ResourceUpdate::getID() const {
                return ResourceUpdate::ID();
            }
            const string ResourceUpdate::getShortName() const {
                return ResourceUpdate::ShortName();
            }
            const string ResourceUpdate::getLongName() const {
                return ResourceUpdate::LongName();
            }
            
            void ResourceUpdate::setUnits(const int32_t &_units) {
                units = _units;
            }

            int32_t ResourceUpdate::getUnits() const {
                return units;
            }

            ostream& ResourceUpdate::operator<<(ostream &out) const {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Serializer> s = sf.getQueryableNetstringsSerializer(out);
                
                s->write(1, units);


                return out;
            }
            
            istream& ResourceUpdate::operator>>(istream &in) {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Deserializer> d = sf.getQueryableNetstringsDeserializer(in);

                d->read(1, units);

                return in;
            }
            
            const string ResourceUpdate::toString() const {
                stringstream sstr;

                sstr << "UpdatedResource#" << units << endl;

                return sstr.str();
            }
        }
    }
}