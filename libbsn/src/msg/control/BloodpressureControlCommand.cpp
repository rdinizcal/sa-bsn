#include "msg/control/BloodpressureControlCommand.hpp"

namespace bsn {
    namespace msg { 
        namespace control {
        
            using namespace std;

            BloodpressureControlCommand::BloodpressureControlCommand() : active(), frequency(), m_avg_size() {}

            BloodpressureControlCommand::BloodpressureControlCommand(const bool &_active, const double &_frequency, const int32_t &_m_avg_size) : active(_active), frequency(_frequency), m_avg_size(_m_avg_size) {}

            BloodpressureControlCommand::~BloodpressureControlCommand() {}
            
            BloodpressureControlCommand::BloodpressureControlCommand(const BloodpressureControlCommand &obj) :
                SerializableData(),
                active(obj.getActive()),
                frequency(obj.getFrequency()),
                m_avg_size(obj.getMovingAverageSize()) {}
            
            BloodpressureControlCommand& BloodpressureControlCommand::operator=(const BloodpressureControlCommand &obj) {
                active = obj.getActive();   
                frequency = obj.getFrequency();
                m_avg_size = obj.getMovingAverageSize();
                return (*this);
            }
            
            int32_t BloodpressureControlCommand::ID() {
                return 903;
            }
            const string BloodpressureControlCommand::ShortName() {
                return "BloodpressureControlCommand";
            }
            const string BloodpressureControlCommand::LongName() {
                return "control.BloodpressureControlCommand";
            }
            
            int32_t BloodpressureControlCommand::getID() const {
                return BloodpressureControlCommand::ID();
            }
            const string BloodpressureControlCommand::getShortName() const {
                return BloodpressureControlCommand::ShortName();
            }
            const string BloodpressureControlCommand::getLongName() const {
                return BloodpressureControlCommand::LongName();
            }
            
            void BloodpressureControlCommand::setActive(const bool &_active) {
                active = _active;
            }

            bool BloodpressureControlCommand::getActive() const {
                return active;
            }

            void BloodpressureControlCommand::setFrequency(const double &_frequency) {
                frequency = _frequency;
            }
            
            double BloodpressureControlCommand::getFrequency() const {
                return frequency;
            }

            void BloodpressureControlCommand::setMovingAverageSize(const int32_t &_m_avg_size){
                m_avg_size = _m_avg_size;
            }

            int32_t BloodpressureControlCommand::getMovingAverageSize() const {
                return m_avg_size;
            }

            ostream& BloodpressureControlCommand::operator<<(ostream &out) const {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Serializer> s = sf.getQueryableNetstringsSerializer(out);
                
                s->write(1, active);
                s->write(2, frequency);
                s->write(3, m_avg_size);

                return out;
            }
            
            istream& BloodpressureControlCommand::operator>>(istream &in) {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Deserializer> d = sf.getQueryableNetstringsDeserializer(in);
                
                d->read(1, active);
                d->read(2, frequency);
                d->read(3, m_avg_size);

                return in;
            }
            
            const string BloodpressureControlCommand::toString() const {
                stringstream sstr;

                sstr << "BloodpressureControlCommand#" << endl;
                sstr << "Activate module: " << active << endl;
                sstr << "Freq: " << frequency << endl;
                sstr << "Moving Avg Size: "  << m_avg_size << endl;

                return sstr.str();
            }
        }
    }
}