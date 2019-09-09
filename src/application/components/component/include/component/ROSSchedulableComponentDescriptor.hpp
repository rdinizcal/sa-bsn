#ifndef ROSSCHEDULABLECOMPONENT_DESCRIPTOR_H
#define ROSSCHEDULABLECOMPONENT_DESCRIPTOR_H

#include <string>

#include "archlib/ROSComponentDescriptor.hpp"

class ROSSchedulableComponentDescriptor : public arch::ROSComponentDescriptor{

    public:
        ROSSchedulableComponentDescriptor(const ROSSchedulableComponentDescriptor &);
        ROSSchedulableComponentDescriptor &operator=(const ROSSchedulableComponentDescriptor &);

    private:
        ROSSchedulableComponentDescriptor();
        virtual ~ROSSchedulableComponentDescriptor();
    
    public:
        void setDeadline(const uint32_t &deadline);
        uint32_t getDeadline() const;

        void setWorstCaseExecutionTime(const std::uint32_t &wce);
        uint32_t getWorstCaseExecutionTime() const;

        void setConnection(const bool &connection);
        bool getConnection() const;

    private:
        uint32_t deadline;
        uint32_t wce;
        bool connection;
};


#endif