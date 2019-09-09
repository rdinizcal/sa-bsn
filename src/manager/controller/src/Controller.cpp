#include "controller/Controller.hpp"

Controller::Controller(const int32_t &argc, char **argv, std::string s) : 
	setpoint(1800),
	info(),
	adapt_frequency(0.1) {}

Controller::Controller() :
	setpoint(1800),
	info(),
	adapt_frequency(0.1) {}
	
Controller::~Controller() {}

void Controller::setUp() {}

bool Controller::getInfo() {
	/*******************************************************
	Function to use a client to pull information from log
	********************************************************/

	ros::NodeHandle info_request_handler;

	ros::ServiceClient info_request;

	info_request = info_request_handler.serviceClient<services::ControllerInfo>("InfoRequest");

	services::ControllerInfo srv;

	if(info_request.call(srv)) {
		ROS_INFO("Got info.");

		if(srv.response.empty == false) {
			for(int i = 0;i < sizeof(srv.response.module_name);i++) {
				CurrentInfo aux_info;

				aux_info.setModuleName(srv.response.module_name[i]);
				aux_info.setBatteryLevel(srv.response.battery_level[i]);
				aux_info.setCost(srv.response.cost[i]);
				aux_info.setFrequency(srv.response.frequency[i]);
				aux_info.setRiskStatus(srv.response.risk_status[i]);

				float battery_life;

				battery_life = (aux_info.getBatteryLevel())/(aux_info.getFrequency()*aux_info.getCost());

				aux_info.setExpectedBatteryLife(battery_life);

				info[aux_info.getModuleName()] = aux_info;
			}
		} 
	} else {
		ROS_INFO("Failed to get info");
		return false;
	}

	return srv.response.empty;
}

int Controller::errorCalculation(int estimated_time) {
	return setpoint - estimated_time;
}

void Controller::reconfigureParameters() {
	/*******************************************************
	Function to call a service to reconfigure scheduler
	parameters
	********************************************************/
	ros::NodeHandle parameter_reconfiguration_handler;

	ros::ServiceClient parameter_reconfigure;

	services::ParameterReconfiguration srv;
	
	std::map<std::string, float>::iterator parameters_it;

	int i = 0;
	
	for(parameters_it = new_parameters.begin();parameters_it != new_parameters.end();++parameters_it) {
		srv.request.module_names[i] = parameters_it->first;
		srv.request.frequencies[i] = parameters_it->second;

		int64_t deadline = (1/static_cast<int64_t>(parameters_it->second - 0.5))*1000000L; //rounding frequency down

		srv.request.deadlines[i] = deadline;
		i++;
	}

	//ParameterReconfiguration srv to be done
	parameter_reconfigure = parameter_reconfiguration_handler.serviceClient<services::ParameterReconfiguration>("ParameterReconfiguration");

	if(parameter_reconfigure.call(srv)) {
		ROS_INFO("Succesfully reconfigured parameters.");
	} else {
		ROS_INFO("Failed to reconfigure parameters.");
	}
}

void Controller::adaptation(CurrentInfo module_info) {
	float k;

	k = -1*(module_info.getBatteryLevel()/(static_cast<float>(setpoint*module_info.getExpectedBatteryLife())*module_info.getCost()));

	//Here the risk_status comes in to give boundaries to the frequency value (When implemented)
	new_parameters[module_info.getModuleName()] = module_info.getFrequency() + static_cast<float>(errorCalculation(static_cast<int>(module_info.getExpectedBatteryLife())))*k; //Delta F
}

void Controller::run() {
	ros::Rate loop_rate(adapt_frequency);
	
	while(ros::ok()) {
		ros::spinOnce();

		if(getInfo()) {
			std::map<std::string, CurrentInfo>::iterator info_it;

			for(info_it = info.begin();info_it != info.end();++info_it) {
				adaptation(info_it->second);
			}

			reconfigureParameters();

			new_parameters.clear();
			info.clear();
		}
		
		loop_rate.sleep();
	}
}