/*
 * Mensagem com dados do sensor remetente
 * 
 * @author Ricardo Diniz Caldas
 * @version v1.0
 */

#include "msg/data/SensorData.h"
#include <iostream>

namespace bsn {
    namespace msg { 
        namespace data {
        
            using namespace std;

            SensorData::SensorData() : type(), data(), risk() {}
            
            SensorData::SensorData(const std::string &_type, const double &_data, const double &_risk) : 
                type(_type),
                data(_data),
                risk(_risk) {}
            
            SensorData::~SensorData() {}
            
            SensorData::SensorData(const SensorData &obj) :
                SerializableData(),
                type(obj.getType()),
                data(obj.getData()),
                risk(obj.getRisk()) {}
            
            SensorData& SensorData::operator=(const SensorData &obj) {
                type = obj.getType();
                data = obj.getData();
                risk = obj.getRisk();           
                return (*this);
            }
            
            int32_t SensorData::ID() {
                return 873;
            }
            const string SensorData::ShortName() {
                return "SensorData";
            }
            const string SensorData::LongName() {
                return "data.SensorData";
            }
            
            int32_t SensorData::getID() const {
                return SensorData::ID();
            }
            const string SensorData::getShortName() const {
                return SensorData::ShortName();
            }
            const string SensorData::getLongName() const {
                return SensorData::LongName();
            }

            void SensorData::setType(const std::string &_type){
                type = _type;
            }

            std::string SensorData::getType() const {
                return type;
            }

            void SensorData::setData(const double &_data) {
                data = _data;
            }

            double SensorData::getData() const {
                return data;
            }

            void SensorData::setRisk(const double &_risk) {
                risk = _risk;
            }

            double SensorData::getRisk() const {
                return risk;
            }

            ostream& SensorData::operator<<(ostream &out) const {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Serializer> s = sf.getQueryableNetstringsSerializer(out);
                
                s->write(1, type);
                s->write(2, data);
                s->write(3, risk);

                return out;
            }
            
            istream& SensorData::operator>>(istream &in) {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Deserializer> d = sf.getQueryableNetstringsDeserializer(in);
                
                d->read(1, type);
                d->read(2, data);
                d->read(3, risk);

                return in;
            }
            
            const string SensorData::toString() const {
                stringstream sstr;

                sstr << "SensorData#" << endl;
                sstr << "type: " << type << endl;
                sstr << "data: " << data << endl;
                sstr << "risk: " << risk << "%" << endl;


                return sstr.str();
            }
        }
    }
}
