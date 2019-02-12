#include "msg/info/PatientStatusInfo.h"

namespace bsn {
    namespace msg { 
        namespace info {
        
            using namespace std;

            PatientStatusInfo::PatientStatusInfo() : 
                thermometerRisk(),
                ecgRisk(),
                oximeterRisk(),
                bloodpressureRisk(),
                patientStatus() {}
            
            PatientStatusInfo::PatientStatusInfo(const std::string &_thermometerRisk, const std::string &_ecgRisk, const std::string &_oximeterRisk, const std::string &_bloodpressureRisk, const std::string &_patientStatus) : 
                thermometerRisk(_thermometerRisk),
                ecgRisk(_ecgRisk),
                oximeterRisk(_oximeterRisk),
                bloodpressureRisk(_bloodpressureRisk),
                patientStatus(_patientStatus) {}
            
            PatientStatusInfo::~PatientStatusInfo() {}
            
            PatientStatusInfo::PatientStatusInfo(const PatientStatusInfo &obj) :
                SerializableData(),
                thermometerRisk(obj.getThermometerRisk()),
                ecgRisk(obj.getECGRisk()),
                oximeterRisk(obj.getOximeterRisk()),
                bloodpressureRisk(obj.getBloodPressureRisk()),
                patientStatus(obj.getPatientStatus()){}
            
            PatientStatusInfo& PatientStatusInfo::operator=(const PatientStatusInfo &obj) {
                thermometerRisk = obj.getThermometerRisk();
                ecgRisk = obj.getECGRisk();
                oximeterRisk = obj.getOximeterRisk();
                bloodpressureRisk = obj.getBloodPressureRisk();
                patientStatus = obj.getPatientStatus();        
                return (*this);
            }
            
            int32_t PatientStatusInfo::ID() {
                return 702;
            }
            const string PatientStatusInfo::ShortName() {
                return "PatientStatusInfo";
            }
            const string PatientStatusInfo::LongName() {
                return "info.PatientStatusInfo";
            }
            
            int32_t PatientStatusInfo::getID() const {
                return PatientStatusInfo::ID();
            }
            const string PatientStatusInfo::getShortName() const {
                return PatientStatusInfo::ShortName();
            }
            const string PatientStatusInfo::getLongName() const {
                return PatientStatusInfo::LongName();
            }

            void PatientStatusInfo::setThermometerRisk(const std::string &_thermometerRisk) {
                thermometerRisk = _thermometerRisk;
            }

            std::string PatientStatusInfo::getThermometerRisk() const{
                return thermometerRisk;
            }

            void PatientStatusInfo::setECGRisk(const std::string &_ecgRisk) {
                ecgRisk = _ecgRisk;
            }

            std::string PatientStatusInfo::getECGRisk() const {
                return ecgRisk;
            }

            void PatientStatusInfo::setOximeterRisk(const std::string &_oximeterRisk){
                oximeterRisk = _oximeterRisk;
            }

            std::string PatientStatusInfo::getOximeterRisk() const {
                return oximeterRisk;
            }

            void PatientStatusInfo::setBloodPressureRisk(const std::string &_bloodpressureRisk){
                bloodpressureRisk = _bloodpressureRisk;
            }

            std::string PatientStatusInfo::getBloodPressureRisk() const {
                return bloodpressureRisk;
            }

            void PatientStatusInfo::setPatientStatus(const std::string &_patientStatus) {
                patientStatus = _patientStatus;
            }

            std::string PatientStatusInfo::getPatientStatus() const {
                return patientStatus;
            }
            
            ostream& PatientStatusInfo::operator<<(ostream &out) const {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Serializer> s = sf.getQueryableNetstringsSerializer(out);
                
                s->write(1, thermometerRisk);
                s->write(2, ecgRisk);
                s->write(3, oximeterRisk);
                s->write(4, bloodpressureRisk);
                s->write(5, patientStatus);

                return out;
            }
            
            istream& PatientStatusInfo::operator>>(istream &in) {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Deserializer> d = sf.getQueryableNetstringsDeserializer(in);
                
                d->read(1, thermometerRisk);
                d->read(2, ecgRisk);
                d->read(3, oximeterRisk);
                d->read(4, bloodpressureRisk);
                d->read(5, patientStatus);

                return in;
            }
            
            const string PatientStatusInfo::toString() const {
                stringstream sstr;

                sstr << "PatientStatusInfo#" << endl;
                sstr << "| THERM_RISK: " << thermometerRisk << endl; 
                sstr << "| ECG_RISK: " << ecgRisk << endl;
                sstr << "| OXIM_RISK: " << oximeterRisk << endl;
                sstr << "| BPRESS_RISK: " << bloodpressureRisk << endl; 
                sstr << "| PATIENT_STATUS: " << patientStatus << endl;

                return sstr.str();
            }
        }
    }
}