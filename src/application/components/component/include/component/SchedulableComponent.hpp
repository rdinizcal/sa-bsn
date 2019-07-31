#ifndef SCHEDULABLE_COMPONENT_HPP
#define SCHEDULABLE_COMPONENT_HPP

#include <signal.h>
#include <string>

#include "archlib/Module.hpp"

#include "std_msgs/String.h"
#include "ros/ros.h"

#include "services/SchedulerServerData.h"
#include "messages/Finish.h"
#include "messages/Init.h"

class SchedulableComponent : public arch::Module {

    private:
        void schedulingCallback(const messages::Init& msg);

    public:
        SchedulableComponent(const int32_t &argc, char **argv);
        virtual ~SchedulableComponent();

        static void schedulingSigIntHandler(int sig);

        virtual void setUp();
        virtual void tearDown();
        void run();
        virtual void body() = 0;

        virtual void sendEvent(const std::string &/*key*/, const std::string &/*value*/) = 0;
        virtual void sendStatus(const std::string &/*key*/, const double &/*value*/) = 0;

        void setReceivedName(const std::string &received_name);
        std::string getReceivedName() const;

        void setCheckFrequency(const double &check_frequency);
        double getCheckFrequency() const;

        void setPeriod(const double &period);
        double getPeriod() const;

    private:
        std::string received_name;
        double check_frequency;
        double period;

        /* communication interface variables */
        ros::Publisher task_finished_pub;
        ros::Subscriber schedule_task;
        ros::ServiceClient client_module;
        std::string topic_name;
        std::string finish_topic_name;
};

#endif