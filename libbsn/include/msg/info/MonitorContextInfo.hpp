#ifndef MONITOR_CONTEXT_INFO_H
#define MONITOR_CONTEXT_INFO_H

#include "opendavinci/odcore/data/SerializableData.h"
#include "opendavinci/odcore/serialization/Deserializer.h"
#include "opendavinci/odcore/serialization/SerializationFactory.h"
#include "opendavinci/odcore/serialization/Serializer.h"

namespace bsn {
    namespace msg { 
        namespace info {
        
            class MonitorContextInfo : public odcore::data::SerializableData {
            
                public:
                    MonitorContextInfo(const std::string &/*context_id*/, const bool &/*_b_value*/, const double &/*_d_value*/, const int32_t &/*_i_value*/, const std::string &/*_s_value*/);
    
                    MonitorContextInfo();
                    virtual ~MonitorContextInfo();
                    
                    MonitorContextInfo(const MonitorContextInfo &);
                    MonitorContextInfo& operator=(const MonitorContextInfo &);
                
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
                    void setContext(const std::string &/*context_id*/);
                    std::string getContext() const;

                    void setBoolValue(const bool &/*_b_value*/);
                    bool getBoolValue() const;

                    void setDoubleValue(const double &/*_d_value*/);
                    double getDoubleValue() const;

                    void setIntegerValue(const int32_t &/*_i_value*/);
                    int32_t getIntegerValue() const;

                    void setStringValue(const std::string &/*_s_value*/);
                    std::string getStringValue() const;
                
                private:
                    std::string context_id;
                    bool        b_value;
                    double      d_value;
                    int32_t     i_value;
                    std::string s_value;
            };
        }
    }
}

#endif