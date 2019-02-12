#ifndef DATA_SENSOR_H_
#define DATA_SENSOR_H_

#include "opendavinci/odcore/data/SerializableData.h"
#include "opendavinci/odcore/serialization/Deserializer.h"
#include "opendavinci/odcore/serialization/SerializationFactory.h"
#include "opendavinci/odcore/serialization/Serializer.h"

#include <stdint.h>

namespace bsn {
    namespace msg { 
        namespace data {
        
            class SensorData : public odcore::data::SerializableData {
            
                public:
                    SensorData(const std::string &/*type*/, const double &/*data*/, const double &/*risk*/);
    
                    SensorData();
                    virtual ~SensorData();
                    
                    SensorData(const SensorData &/*obj*/);
                    SensorData& operator=(const SensorData &/*obj*/);
                
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
                    
                public:
                    void setType(const std::string &/*type*/);
                    std::string getType() const;

                    void setData(const double &/*data*/);
                    double getData() const;

                    void setRisk(const double &/*risk*/);
                    double getRisk() const;

                private:
                    std::string type;
                    double data;
                    double risk;
            };
        }
    }
}

#endif