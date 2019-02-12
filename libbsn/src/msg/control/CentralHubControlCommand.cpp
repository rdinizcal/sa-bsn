#include "msg/control/CentralHubControlCommand.hpp"

namespace bsn {
    namespace msg { 
        namespace control {
        
            using namespace std;

            CentralHubControlCommand::CentralHubControlCommand() : active(), frequency() {}

            CentralHubControlCommand::CentralHubControlCommand(const bool &_active, const double &_frequency) : active(_active), frequency(_frequency) {}

            CentralHubControlCommand::~CentralHubControlCommand() {}
            
            CentralHubControlCommand::CentralHubControlCommand(const CentralHubControlCommand &obj) :
                SerializableData(),
                active(obj.getActive()),
                frequency(obj.getFrequency()) {}
            
            CentralHubControlCommand& CentralHubControlCommand::operator=(const CentralHubControlCommand &obj) {
                active = obj.getActive();   
                frequency = obj.getFrequency();
                return (*this);
            }
            
            int32_t CentralHubControlCommand::ID() {
                return 904;
            }
            const string CentralHubControlCommand::ShortName() {
                return "CentralHubControlCommand";
            }
            const string CentralHubControlCommand::LongName() {
                return "control.CentralHubControlCommand";
            }
            
            int32_t CentralHubControlCommand::getID() const {
                return CentralHubControlCommand::ID();
            }
            const string CentralHubControlCommand::getShortName() const {
                return CentralHubControlCommand::ShortName();
            }
            const string CentralHubControlCommand::getLongName() const {
                return CentralHubControlCommand::LongName();
            }
            
            void CentralHubControlCommand::setActive(const bool &_active) {
                active = _active;
            }

            bool CentralHubControlCommand::getActive() const {
                return active;
            }

            void CentralHubControlCommand::setFrequency(const double &_frequency) {
                frequency = _frequency;
            }
            
            double CentralHubControlCommand::getFrequency() const {
                return frequency;
            }

            ostream& CentralHubControlCommand::operator<<(ostream &out) const {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Serializer> s = sf.getQueryableNetstringsSerializer(out);
                
                s->write(1, active);
                s->write(2, frequency);

                return out;
            }
            
            istream& CentralHubControlCommand::operator>>(istream &in) {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Deserializer> d = sf.getQueryableNetstringsDeserializer(in);
                
                d->read(1, active);
                d->read(2, frequency);

                return in;
            }
            
            const string CentralHubControlCommand::toString() const {
                stringstream sstr;

                sstr << "CentralHubControlCommand#" << endl;
                sstr << "Activate module: " << active << endl;
                sstr << "Freq: " << frequency << endl;

                return sstr.str();
            }
        }
    }
}