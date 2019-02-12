#ifndef PATIENT_STATUS_INFO_H
#define PATIENT_STATUS_INFO_H

#include "opendavinci/odcore/data/SerializableData.h"
#include "opendavinci/odcore/serialization/Deserializer.h"
#include "opendavinci/odcore/serialization/SerializationFactory.h"
#include "opendavinci/odcore/serialization/Serializer.h"

#include <stdint.h>

#include "bsn/resource/Resource.hpp"

namespace bsn {
    namespace msg { 
        namespace info {
        
            class PatientStatusInfo : public odcore::data::SerializableData {
            
                public:
                    PatientStatusInfo(const std::string &/*thermometer risk*/, const std::string &/*ecg risk*/, const std::string &/*oximeter risk*/, const std::string &/*blood pressure risk*/, const std::string &/*patient status*/);
    
                    PatientStatusInfo(); // construtor
                    virtual ~PatientStatusInfo();// destrutor 
                    
                    //Boas praticas do manual do OpenDaVINCI
                    PatientStatusInfo(const PatientStatusInfo &/*obj*/);
                    PatientStatusInfo& operator=(const PatientStatusInfo &/*obj*/);
                
                // Métodos abstratos
                public:
                    virtual int32_t getID() const;
                    virtual const std::string getShortName() const;
                    virtual const std::string getLongName() const;

                    static int32_t ID();
                    static const std::string ShortName();
                    static const std::string LongName();
            
                public:
                    virtual std::ostream& operator<<(std::ostream &out) const;
                    virtual std::istream& operator>>(std::istream &in);
            
                    virtual const std::string toString() const;
                    
                // setters e getters
                public:
                    void setThermometerRisk(const std::string &/*thermometer risk*/);
                    std::string getThermometerRisk() const;

                    void setECGRisk(const std::string &/*ecg risk*/);
                    std::string getECGRisk() const;

                    void setOximeterRisk(const std::string &/*oximeter risk*/);
                    std::string getOximeterRisk() const;

                    void setBloodPressureRisk(const std::string &/*blood pressure risk*/);
                    std::string getBloodPressureRisk() const;

                    void setPatientStatus(const std::string &/*patient status*/);
                    std::string getPatientStatus() const;
                
                private:
                    std::string thermometerRisk;
                    std::string ecgRisk;
                    std::string oximeterRisk;
                    std::string bloodpressureRisk;
                    std::string patientStatus;
            };
        }
    }
}

#endif