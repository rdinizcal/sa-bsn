#include "msg/control/OximeterControlCommand.hpp"

namespace bsn {
    namespace msg { 
        namespace control {
        
            using namespace std;

            OximeterControlCommand::OximeterControlCommand() : active(), frequency(), m_avg_size() {}

            OximeterControlCommand::OximeterControlCommand(const bool &_active, const double &_frequency, const int32_t &_m_avg_size) : active(_active), frequency(_frequency), m_avg_size(_m_avg_size) {}

            OximeterControlCommand::~OximeterControlCommand() {}
            
            OximeterControlCommand::OximeterControlCommand(const OximeterControlCommand &obj) :
                SerializableData(),
                active(obj.getActive()),
                frequency(obj.getFrequency()),
                m_avg_size(obj.getMovingAverageSize()){}
            
            OximeterControlCommand& OximeterControlCommand::operator=(const OximeterControlCommand &obj) {
                active = obj.getActive();   
                frequency = obj.getFrequency();
                m_avg_size = obj.getMovingAverageSize();     
                return (*this);
            }
            
            int32_t OximeterControlCommand::ID() {
                return 901;
            }
            const string OximeterControlCommand::ShortName() {
                return "OximeterControlCommand";
            }
            const string OximeterControlCommand::LongName() {
                return "control.OximeterControlCommand";
            }
            
            int32_t OximeterControlCommand::getID() const {
                return OximeterControlCommand::ID();
            }
            const string OximeterControlCommand::getShortName() const {
                return OximeterControlCommand::ShortName();
            }
            const string OximeterControlCommand::getLongName() const {
                return OximeterControlCommand::LongName();
            }
            
            void OximeterControlCommand::setActive(const bool &_active) {
                active = _active;
            }

            bool OximeterControlCommand::getActive() const {
                return active;
            }

            void OximeterControlCommand::setFrequency(const double &_frequency) {
                frequency = _frequency;
            }
            
            double OximeterControlCommand::getFrequency() const {
                return frequency;
            }

            void OximeterControlCommand::setMovingAverageSize(const int32_t &_m_avg_size){
                m_avg_size = _m_avg_size;
            }

            int32_t OximeterControlCommand::getMovingAverageSize() const {
                return m_avg_size;
            }

            ostream& OximeterControlCommand::operator<<(ostream &out) const {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Serializer> s = sf.getQueryableNetstringsSerializer(out);
                
                s->write(1, active);
                s->write(2, frequency);
                s->write(3, m_avg_size);

                return out;
            }
            
            istream& OximeterControlCommand::operator>>(istream &in) {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Deserializer> d = sf.getQueryableNetstringsDeserializer(in);
                
                d->read(1, active);
                d->read(2, frequency);
                d->read(3, m_avg_size);

                return in;
            }
            
            const string OximeterControlCommand::toString() const {
                stringstream sstr;

                sstr << "OximeterControlCommand#" << endl;
                sstr << "Activate module: " << active << endl;
                sstr << "Freq: " << frequency << endl;
                sstr << "Moving Avg Size: "  << m_avg_size << endl;

                return sstr.str();
            }
        }
    }
}