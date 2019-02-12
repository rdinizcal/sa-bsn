#include "msg/control/ThermometerControlCommand.hpp"

namespace bsn {
    namespace msg { 
        namespace control {
        
            using namespace std;

            ThermometerControlCommand::ThermometerControlCommand() : active(), frequency(), m_avg_size() {}

            ThermometerControlCommand::ThermometerControlCommand(const bool &_active, const double &_frequency, const int32_t &_m_avg_size) : active(_active), frequency(_frequency), m_avg_size(_m_avg_size) {}

            ThermometerControlCommand::~ThermometerControlCommand() {}
            
            ThermometerControlCommand::ThermometerControlCommand(const ThermometerControlCommand &obj) :
                SerializableData(),
                active(obj.getActive()),
                frequency(obj.getFrequency()),
                m_avg_size(obj.getMovingAverageSize()) {}
            
            ThermometerControlCommand& ThermometerControlCommand::operator=(const ThermometerControlCommand &obj) {
                active = obj.getActive();   
                frequency = obj.getFrequency();
                m_avg_size = obj.getMovingAverageSize();
                return (*this);
            }
            
            int32_t ThermometerControlCommand::ID() {
                return 900;
            }
            const string ThermometerControlCommand::ShortName() {
                return "ThermometerControlCommand";
            }
            const string ThermometerControlCommand::LongName() {
                return "control.ThermometerControlCommand";
            }
            
            int32_t ThermometerControlCommand::getID() const {
                return ThermometerControlCommand::ID();
            }
            const string ThermometerControlCommand::getShortName() const {
                return ThermometerControlCommand::ShortName();
            }
            const string ThermometerControlCommand::getLongName() const {
                return ThermometerControlCommand::LongName();
            }
            
            void ThermometerControlCommand::setActive(const bool &_active) {
                active = _active;
            }

            bool ThermometerControlCommand::getActive() const {
                return active;
            }

            void ThermometerControlCommand::setFrequency(const double &_frequency) {
                frequency = _frequency;
            }
            
            double ThermometerControlCommand::getFrequency() const {
                return frequency;
            }

            void ThermometerControlCommand::setMovingAverageSize(const int32_t &_m_avg_size){
                m_avg_size = _m_avg_size;
            }

            int32_t ThermometerControlCommand::getMovingAverageSize() const {
                return m_avg_size;
            }

            ostream& ThermometerControlCommand::operator<<(ostream &out) const {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Serializer> s = sf.getQueryableNetstringsSerializer(out);
                
                s->write(1, active);
                s->write(2, frequency);
                s->write(3, m_avg_size);

                return out;
            }
            
            istream& ThermometerControlCommand::operator>>(istream &in) {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Deserializer> d = sf.getQueryableNetstringsDeserializer(in);
                
                d->read(1, active);
                d->read(2, frequency);
                d->read(3, m_avg_size);

                return in;
            }
            
            const string ThermometerControlCommand::toString() const {
                stringstream sstr;

                sstr << "ThermometerControlCommand#" << endl;
                sstr << "Activate module: " << active << endl;
                sstr << "Freq: " << frequency << endl;
                sstr << "Moving Avg Size: "  << m_avg_size << endl;

                return sstr.str();
            }
        }
    }
}