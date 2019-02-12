#ifndef CENTRALHUB_CONTROL_COMMAND_H
#define CENTRALHUB_CONTROL_COMMAND_H

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
            
            class CentralHubControlCommand : public odcore::data::SerializableData {
            
                public:
                    CentralHubControlCommand(const bool &/*active*/, const double &/*frequency*/);
    
                    CentralHubControlCommand();        
                    virtual ~CentralHubControlCommand();
                    
                    CentralHubControlCommand(const CentralHubControlCommand &/*obj*/);
                    CentralHubControlCommand& operator=(const CentralHubControlCommand &/*obj*/);
                
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

                private:
                    bool active;
                    double frequency;
            };
        }
    }
}

#endif