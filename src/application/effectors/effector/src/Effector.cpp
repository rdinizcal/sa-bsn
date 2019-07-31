#include "effector/Effector.hpp"

Effector::Effector(int  &argc, char **argv) {}
Effector::~Effector() {}

void Effector::setUp() {
	register_service = handler.advertiseService("EffectorRegister", &Effector::moduleConnect, this);
}

void Effector::receiveReconfigurationCommand(const messages::ReconfigurationCommand::ConstPtr& msg) {
	if (reconfigure_pub.find(msg->target) != reconfigure_pub.end()){
    	reconfigure_pub[msg->target].publish(msg);
	} else {
		ROS_INFO("ERROR, target not found! [%s]", msg->target.c_str());
	}
}

bool Effector::moduleConnect(services::EffectorRegister::Request &req, services::EffectorRegister::Response &res) {

	try {
		if(req.connection == true) {

            ros::Publisher pub = handler.advertise<messages::ReconfigurationCommand>("effect_" + req.name, 1);
            reconfigure_pub[req.name] = pub;

			ROS_INFO("Module Connected. [%s]", req.name.c_str());

		} else {

			std::map<std::string,ros::Publisher>::iterator it;
			it = reconfigure_pub.find(req.name);
			reconfigure_pub.erase(it);

			ROS_INFO("Module Disconnected. [%s]", req.name.c_str());
		}

		res.ACK = true;

	} catch(...) {
		res.ACK = false;
	}

	return true;
}

void Effector::run(){
    ros::NodeHandle n;
    ros::Subscriber reconfigure_sub = n.subscribe("reconfigure", 1000, &Effector::receiveReconfigurationCommand, this);
    ros::spin();
}
