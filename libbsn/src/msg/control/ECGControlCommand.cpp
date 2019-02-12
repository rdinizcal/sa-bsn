#include "msg/control/ECGControlCommand.hpp"

namespace bsn {
    namespace msg { 
        namespace control {
        
            using namespace std;

            ECGControlCommand::ECGControlCommand() : active(), frequency(), m_avg_size() {}
            
            ECGControlCommand::ECGControlCommand(const bool &_active, const double &_frequency, const int32_t &_m_avg_size) : active (_active), frequency(_frequency), m_avg_size(_m_avg_size) {}
            
            ECGControlCommand::~ECGControlCommand() {}
            
            ECGControlCommand::ECGControlCommand(const ECGControlCommand &obj) :
                SerializableData(),
                active(obj.getActive()),
                frequency(obj.getFrequency()),
                m_avg_size(obj.getMovingAverageSize()) {}
            
            ECGControlCommand& ECGControlCommand::operator=(const ECGControlCommand &obj) {
                active = obj.getActive();   
                frequency = obj.getFrequency();
                m_avg_size = obj.getMovingAverageSize();                     
                return (*this);
            }
            
            int32_t ECGControlCommand::ID() {
                return 902;
            }
            const string ECGControlCommand::ShortName() {
                return "ECGControlCommand";
            }
            const string ECGControlCommand::LongName() {
                return "control.ECGControlCommand";
            }
            
            int32_t ECGControlCommand::getID() const {
                return ECGControlCommand::ID();
            }
            const string ECGControlCommand::getShortName() const {
                return ECGControlCommand::ShortName();
            }
            const string ECGControlCommand::getLongName() const {
                return ECGControlCommand::LongName();
            }
            
            void ECGControlCommand::setActive(const bool &_active) {
                active = _active;
            }

            bool ECGControlCommand::getActive() const {
                return active;
            }

            void ECGControlCommand::setFrequency(const double &_frequency) {
                frequency = _frequency;
            }
            
            double ECGControlCommand::getFrequency() const {
                return frequency;
            }

            void ECGControlCommand::setMovingAverageSize(const int32_t &_m_avg_size){
                m_avg_size = _m_avg_size;
            }

            int32_t ECGControlCommand::getMovingAverageSize() const {
                return m_avg_size;
            }

            ostream& ECGControlCommand::operator<<(ostream &out) const {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Serializer> s = sf.getQueryableNetstringsSerializer(out);
                
                s->write(1, active);
                s->write(2, frequency);
                s->write(3, m_avg_size);

                return out;
            }
            
            istream& ECGControlCommand::operator>>(istream &in) {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Deserializer> d = sf.getQueryableNetstringsDeserializer(in);
                
                d->read(1, active);
                d->read(2, frequency);
                d->read(3, m_avg_size);

                return in;
            }
            
            const string ECGControlCommand::toString() const {
                stringstream sstr;

                sstr << "ECGControlCommand#" << endl;
                sstr << "Activate module: " << active << endl;
                sstr << "Freq: " << frequency << endl;
                sstr << "Moving Avg Size: "  << m_avg_size << endl;
                
                return sstr.str();
            }
        }
    }
}