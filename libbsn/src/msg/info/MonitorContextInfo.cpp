#include "msg/info/MonitorContextInfo.hpp"

namespace bsn {
    namespace msg { 
        namespace info {
        
            using namespace std;

            MonitorContextInfo::MonitorContextInfo() : context_id(), b_value(), d_value(), i_value(), s_value() {}
            
            MonitorContextInfo::MonitorContextInfo(const string &_context_id, const bool &_b_value, const double &_d_value, const int32_t &_i_value, const string &_s_value) : context_id(_context_id), b_value(_b_value), d_value(_d_value), i_value(_i_value), s_value(_s_value) {}

            MonitorContextInfo::~MonitorContextInfo() {}
            
            MonitorContextInfo::MonitorContextInfo(const MonitorContextInfo &obj) :
                SerializableData(),
                context_id(obj.getContext()),
                b_value(obj.getBoolValue()),
                d_value(obj.getDoubleValue()),
                i_value(obj.getIntegerValue()),
                s_value(obj.getStringValue()) {}
            
            MonitorContextInfo& MonitorContextInfo::operator=(const MonitorContextInfo &obj) {
                context_id = obj.getContext();
                b_value = obj.getBoolValue();
                d_value = obj.getDoubleValue();
                i_value = obj.getIntegerValue();
                s_value = obj.getStringValue();
                return (*this);
            }
            
            int32_t MonitorContextInfo::ID() {
                return 707;
            }
            const string MonitorContextInfo::ShortName() {
                return "MonitorContextInfo";
            }
            const string MonitorContextInfo::LongName() {
                return "info.MonitorContextInfo";
            }
            
            int32_t MonitorContextInfo::getID() const {
                return MonitorContextInfo::ID();
            }
            const string MonitorContextInfo::getShortName() const {
                return MonitorContextInfo::ShortName();
            }
            const string MonitorContextInfo::getLongName() const {
                return MonitorContextInfo::LongName();
            }

            void MonitorContextInfo::setContext(const string &_context_id) {
                context_id = _context_id;
            }

            string MonitorContextInfo::getContext() const{
                return context_id;
            }

            void MonitorContextInfo::setBoolValue(const bool &_b_value) {
                b_value = _b_value;
            }

            bool MonitorContextInfo::getBoolValue() const {
                return b_value;
            }

            void MonitorContextInfo::setDoubleValue(const double &_d_value) {
                d_value = _d_value;
            }

            double MonitorContextInfo::getDoubleValue() const {
                return d_value;
            }

            void MonitorContextInfo::setIntegerValue(const int32_t &_i_value) {
                i_value = _i_value;
            }

            int32_t MonitorContextInfo::getIntegerValue() const {
                return i_value;
            }

            void MonitorContextInfo::setStringValue(const std::string &_s_value) {
                s_value= _s_value;
            }

            std::string MonitorContextInfo::getStringValue() const{
                return s_value;
            }

            ostream& MonitorContextInfo::operator<<(ostream &out) const {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                shared_ptr<odcore::serialization::Serializer> s = sf.getQueryableNetstringsSerializer(out);
                
                s->write(1, context_id);
                s->write(2, b_value);
                s->write(3, d_value);
                s->write(4, i_value);
                s->write(5, s_value);


                return out;
            }
            
            istream& MonitorContextInfo::operator>>(istream &in) {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                shared_ptr<odcore::serialization::Deserializer> d = sf.getQueryableNetstringsDeserializer(in);
                
                d->read(1, context_id);
                d->read(2, b_value);
                d->read(3, d_value);
                d->read(4, i_value);
                d->read(5, s_value);

                return in;
            }
            
            const string MonitorContextInfo::toString() const {
                stringstream sstr;

                sstr << "MonitorContextInfo#" << endl;
                sstr << "Id: " << context_id << endl; 
                sstr << "bool: " << b_value << endl; 
                sstr << "double: " << d_value << endl; 
                sstr << "int: " << i_value << endl; 
                sstr << "string: " << s_value << endl; 

                return sstr.str();
            }
        }
    }
}