#include "param_adapter/ParamAdapter.hpp"

ParamAdapter::ParamAdapter(int  &argc, char **argv, const std::string &name) : Effector(argc, argv, name) {}
ParamAdapter::~ParamAdapter() {}

void ParamAdapter::setUp() {
	register_service = handle.advertiseService("EffectorRegister", &ParamAdapter::moduleConnect, this);
			
	double freq;
	handle.getParam("frequency", freq);
	rosComponentDescriptor.setFreq(freq);
}

void ParamAdapter::tearDown() {}

void ParamAdapter::body() {
	ros::NodeHandle n;
	ros::Subscriber reconf = n.subscribe("reconfigure", 1000, &ParamAdapter::receiveAdaptationCommand, this);
	ros::spin();

}

void ParamAdapter::receiveAdaptationCommand(const archlib::AdaptationCommand::ConstPtr& msg) {
	if (target_arr.find(msg->target) != target_arr.end()){
    	target_arr[msg->target].publish(msg);
	} else {
		ROS_INFO("ERROR, target not found! [%s]", msg->target.c_str());
	}
}

bool ParamAdapter::moduleConnect(archlib::EffectorRegister::Request &req, archlib::EffectorRegister::Response &res) {

	try {
		if(req.connection == true) {

            ros::Publisher pub = handle.advertise<archlib::AdaptationCommand>("reconfigure_" + req.name, 1);
            target_arr[req.name] = pub;

			ROS_INFO("Module Connected. [%s]", req.name.c_str());

		} else {

			std::map<std::string,ros::Publisher>::iterator it;
			it = target_arr.find(req.name);
			target_arr.erase(it);

			ROS_INFO("Module Disconnected. [%s]", req.name.c_str());
		}

		res.ACK = true;

	} catch(...) {
		res.ACK = false;
	}

	return true;
}