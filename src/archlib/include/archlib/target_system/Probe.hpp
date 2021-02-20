#ifndef PROBE_HPP
#define PROBE_HPP

#include "ros/ros.h"

#include "archlib/Event.h"
#include "archlib/Status.h"
#include "archlib/EnergyStatus.h"
#include "archlib/ROSComponent.hpp"

namespace arch {
    namespace target_system {

        class Probe : public ROSComponent {

            public:
                Probe(int &argc, char **argv, const std::string &name);
                virtual ~Probe();

            private:
                Probe(const Probe &);
                Probe &operator=(const Probe &);

            public:
                virtual void setUp();
                virtual void tearDown();
                virtual void body();

                void collectEvent(const archlib::Event::ConstPtr& /*msg*/);
                void collectStatus(const archlib::Status::ConstPtr& /*msg*/);
                void collectEnergyStatus(const archlib::EnergyStatus::ConstPtr& /*msg*/);

            protected:
                ros::NodeHandle handle;

            private:
                ros::Subscriber collect_event;
                ros::Subscriber collect_status;
                ros::Subscriber collect_energy_status;
                ros::Publisher log_event;
                ros::Publisher log_status;
                ros::Publisher log_energy_status;
        };

    }
}


#endif 