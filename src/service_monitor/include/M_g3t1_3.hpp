#include <list>
#include "ros/ros.h"


class Monitor_g3t1_3 (){

    public:
        Monitor_g3t1_3 ();
        ~Monitor_g3t1_3 ();

    private:
        ros::NodeHandle serviceInput;
        ros::Publisher serviceValidation;
        std::list<archlib::Status::ConstPtr> input_queue;
}