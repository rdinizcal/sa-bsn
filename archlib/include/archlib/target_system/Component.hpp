#ifndef COMPONENT_HPP
#define COMPONENT_HPP

#include <string>
#include <signal.h>

#include "ros/ros.h"

#include "archlib/Event.h"
#include "archlib/Status.h"
#include "archlib/EnergyStatus.h"
#include "archlib/AdaptationCommand.h"
#include "archlib/EffectorRegister.h"

#include "archlib/ROSComponent.hpp"

namespace arch {
    namespace target_system {

        class Component : public ROSComponent {

            public:
                Component(int &argc, char **argv, const std::string &name);
                virtual ~Component();

            private:
                Component(const Component &);
                Component &operator=(const Component &);

            public:
                virtual void setUp();
                virtual void tearDown();
                virtual int32_t run();
                virtual void body() = 0;

                void sendEvent(const std::string &content);
		        void sendStatus(const std::string &content);
                void sendEnergyStatus(const double &cost);

                virtual void reconfigure(const archlib::AdaptationCommand::ConstPtr& msg);

            protected:
                void activate();
                void deactivate();
                static void shutdownComponent();

                ros::NodeHandle handle;
                static void sigIntHandler(int signal);

            private:
                bool status;
                ros::Publisher collect_event;
                ros::Publisher collect_status;
                ros::Publisher collect_energy_status;
                ros::Subscriber effect;
        };

    }
}

#endif
