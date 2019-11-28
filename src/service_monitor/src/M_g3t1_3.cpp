#include "M_g3t1_3.hpp"


M_g3t1_3::M_g3t1_3(int  &argc, char **argv, const std::string &name) : ROSComponent(argc, argv, name), time_ref() {}
M_g3t1_3::~M_g3t1_3() {}

int64_t M_g3t1_3::now() const{
    return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

void M_g3t1_3::monitorStart() {
    time_ref = this->now();

    status = handle.advertise<archlib::Status>("g3t1_3", 1000);

    double freq;
	handle.getParam("frequency", freq);
	rosComponentDescriptor.setFreq(freq);
}

void M_g3t1_3::serviceMonitoring() {
    ros::NodeHandle n;
    ros::Subscriber status_sub = n.subscribe("status", 1000, &M_g3t1_3::receiveStatus, this);
    ros::spin();
}

void M_g3t1_3::receiveStatus(const archlib::Status::ConstPtr& msg) {    

    


    if (M_g3t1_3.input_queue.size() <=2){
        
    
    }
    else {
        persistMsg.source = msg->source;
        persistMsg.target = msg->target;
        persistMsg.type = "Status";
        persistMsg.timestamp = this->now()-time_ref;
        persistMsg.content = msg->content;

        persist.publish(persistMsg);
        status.publish(msg);
    }
    
}