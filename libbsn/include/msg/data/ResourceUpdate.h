#ifndef RESOURCE_UPDATE_H
#define RESOURCE_UPDATE_H

#include "opendavinci/odcore/data/SerializableData.h"
#include "opendavinci/odcore/serialization/Deserializer.h"
#include "opendavinci/odcore/serialization/SerializationFactory.h"
#include "opendavinci/odcore/serialization/Serializer.h"

#include <stdint.h>

namespace bsn {
    namespace msg { 
        namespace data {
        
            class ResourceUpdate : public odcore::data::SerializableData {
            
                public:
                    ResourceUpdate(const int32_t &/*units*/);
    
                    ResourceUpdate(); // construtor
                    virtual ~ResourceUpdate();// destrutor 
                    
                    //Boas praticas do manual do OpenDaVINCI
                    ResourceUpdate(const ResourceUpdate &/*obj*/);
                    ResourceUpdate& operator=(const ResourceUpdate &/*obj*/);
                
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
                    void setUnits(const int32_t &/*units*/);
                    int32_t getUnits() const;
                
                private:
                    int32_t units;
            };
        }
    }
}

#endif