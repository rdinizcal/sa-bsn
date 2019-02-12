#ifndef OXIMETER_CONTROL_COMMAND_H
#define OXIMETER_CONTROL_COMMAND_H

#include "opendavinci/odcore/data/SerializableData.h"
#include "opendavinci/odcore/serialization/Deserializer.h"
#include "opendavinci/odcore/serialization/SerializationFactory.h"
#include "opendavinci/odcore/serialization/Serializer.h"

#include <stdint.h>
#include <map>
#include <sstream>

namespace bsn {
    namespace msg {
        namespace control {
            
            class OximeterControlCommand : public odcore::data::SerializableData {
            
                public:
                    OximeterControlCommand(const bool &/*active*/, const double &/*frequency*/, const int32_t &/*m_avg_size*/);
    
                    OximeterControlCommand();        
                    virtual ~OximeterControlCommand();
                    
                    OximeterControlCommand(const OximeterControlCommand &/*obj*/);
                    OximeterControlCommand& operator=(const OximeterControlCommand &/*obj*/);
                
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
                    void setActive(const bool &/*active*/);
                    bool getActive() const;

                    void setFrequency(const double &/*frequency*/);
                    double getFrequency() const;

                    void setMovingAverageSize(const int32_t &/*m_avg_size*/);
                    int32_t getMovingAverageSize() const;
                
                private:
                    bool active;
                    double frequency;
                    int32_t m_avg_size;
            };
        }
    }
}

#endif