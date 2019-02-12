#include "msg/info/ContextInfo.hpp"

namespace bsn {
    namespace msg { 
        namespace info {
        
            using namespace std;

            ContextInfo::ContextInfo() : context_id(), b_value(), d_value(), i_value(), s_value() {}
            
            ContextInfo::ContextInfo(const string &_context_id, const bool &_b_value, const double &_d_value, const int32_t &_i_value, const string &_s_value) : context_id(_context_id), b_value(_b_value), d_value(_d_value), i_value(_i_value), s_value(_s_value) {}

            ContextInfo::~ContextInfo() {}
            
            ContextInfo::ContextInfo(const ContextInfo &obj) :
                SerializableData(),
                context_id(obj.getContext()),
                b_value(obj.getBoolValue()),
                d_value(obj.getDoubleValue()),
                i_value(obj.getIntegerValue()),
                s_value(obj.getStringValue()) {}
            
            ContextInfo& ContextInfo::operator=(const ContextInfo &obj) {
                context_id = obj.getContext();
                b_value = obj.getBoolValue();
                d_value = obj.getDoubleValue();
                i_value = obj.getIntegerValue();
                s_value = obj.getStringValue();
                return (*this);
            }
            
            int32_t ContextInfo::ID() {
                return 701;
            }
            const string ContextInfo::ShortName() {
                return "ContextInfo";
            }
            const string ContextInfo::LongName() {
                return "info.ContextInfo";
            }
            
            int32_t ContextInfo::getID() const {
                return ContextInfo::ID();
            }
            const string ContextInfo::getShortName() const {
                return ContextInfo::ShortName();
            }
            const string ContextInfo::getLongName() const {
                return ContextInfo::LongName();
            }

            void ContextInfo::setContext(const string &_context_id) {
                context_id = _context_id;
            }

            string ContextInfo::getContext() const{
                return context_id;
            }

            void ContextInfo::setBoolValue(const bool &_b_value) {
                b_value = _b_value;
            }

            bool ContextInfo::getBoolValue() const {
                return b_value;
            }

            void ContextInfo::setDoubleValue(const double &_d_value) {
                d_value = _d_value;
            }

            double ContextInfo::getDoubleValue() const {
                return d_value;
            }

            void ContextInfo::setIntegerValue(const int32_t &_i_value) {
                i_value = _i_value;
            }

            int32_t ContextInfo::getIntegerValue() const {
                return i_value;
            }

            void ContextInfo::setStringValue(const std::string &_s_value) {
                s_value= _s_value;
            }

            std::string ContextInfo::getStringValue() const{
                return s_value;
            }

            ostream& ContextInfo::operator<<(ostream &out) const {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                shared_ptr<odcore::serialization::Serializer> s = sf.getQueryableNetstringsSerializer(out);
                
                s->write(1, context_id);
                s->write(2, b_value);
                s->write(3, d_value);
                s->write(4, i_value);
                s->write(5, s_value);


                return out;
            }
            
            istream& ContextInfo::operator>>(istream &in) {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                shared_ptr<odcore::serialization::Deserializer> d = sf.getQueryableNetstringsDeserializer(in);
                
                d->read(1, context_id);
                d->read(2, b_value);
                d->read(3, d_value);
                d->read(4, i_value);
                d->read(5, s_value);

                return in;
            }
            
            const string ContextInfo::toString() const {
                stringstream sstr;

                sstr << "ContextInfo#" << endl;
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