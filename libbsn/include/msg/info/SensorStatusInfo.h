#ifndef SENSOR_STATUS_INFO_H
#define SENSOR_STATUS_INFO_H

#include "opendavinci/odcore/data/SerializableData.h"
#include "opendavinci/odcore/serialization/Deserializer.h"
#include "opendavinci/odcore/serialization/SerializationFactory.h"
#include "opendavinci/odcore/serialization/Serializer.h"

#include <stdint.h>

namespace bsn {
    namespace msg { 
        namespace info {
        
            class SensorStatusInfo : public odcore::data::SerializableData {
            
                public:
                    SensorStatusInfo(const double &/*value*/);
    
                    SensorStatusInfo(); // construtor
                    virtual ~SensorStatusInfo();// destrutor 
                    
                    //Boas praticas do manual do OpenDaVINCI
                    SensorStatusInfo(const SensorStatusInfo &/*obj*/);
                    SensorStatusInfo& operator=(const SensorStatusInfo &/*obj*/);
                
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
                    void setValue(const double &/*value*/);
                    double getValue() const;
                
                private:
                    double value;

            };
        }
    }
}

#endif