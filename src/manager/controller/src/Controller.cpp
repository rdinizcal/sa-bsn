#include "controller/Controller.hpp"

Controller::Controller(const int32_t &argc, char **argv, std::string s) : 
	setpoint(1800),
	info(),
	adapt_frequency(0.5),
	empty(true) {}

Controller::Controller() :
	setpoint(1800),
	info(),
	adapt_frequency(0.5),
	empty(true) {}
	
Controller::~Controller() {}

void Controller::setUp() {}

void Controller::getInfo(const messages::ControllerInfo::ConstPtr& msg) {
	if(!msg->empty) {
		empty = false;
		ROS_INFO("Getting info");
		std::cout << "\n\n";
		std::cout << "*************************************\n";
		for(std::size_t i = 0;i < msg->module_name.size();i++) {
			
			CurrentInfo aux_info;
			aux_info.setModuleName(msg->module_name[i]);
			ROS_INFO("Name: %s", msg->module_name[i].c_str());
			aux_info.setBatteryLevel(msg->battery_level[i]);
			ROS_INFO("Battery: %.2f", msg->battery_level[i]);
			aux_info.setCost(msg->cost[i]);
			ROS_INFO("Cost: %.2f", msg->cost[i]);
			aux_info.setFrequency(msg->frequency[i]);
			ROS_INFO("Frequency: %.2f", msg->frequency[i]);
			aux_info.setRiskStatus(msg->risk_status[i]);
			ROS_INFO("Risk: %s", msg->risk_status[i].c_str());

			float battery_life;

			battery_life = (aux_info.getBatteryLevel())/(aux_info.getFrequency()*aux_info.getCost());

			aux_info.setExpectedBatteryLife(battery_life);
			ROS_INFO("ExpectedBatteryLife: %.2f", battery_life);
			std::cout << "*************************************\n";

			info[aux_info.getModuleName()] = aux_info;
		}
	} else {
		empty = true;
	}
	std::cout << "\n\n";
	ROS_INFO("Got info");
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
	
	for(parameters_it = new_parameters.begin();parameters_it != new_parameters.end();++parameters_it) {
		srv.request.module_names.push_back(parameters_it->first);
		srv.request.frequencies.push_back(parameters_it->second);

		int64_t deadline = static_cast<int64_t>((1/(parameters_it->second))*1000000L); //rounding frequency down

		srv.request.deadlines.push_back(deadline);
	}

	//ParameterReconfiguration srv to be done
	parameter_reconfigure = parameter_reconfiguration_handler.serviceClient<services::ParameterReconfiguration>("ParameterReconfiguration");

	if(parameter_reconfigure.call(srv)) {
		std::cout << "\n\n";
		ROS_INFO("Succesfully reconfigured parameters.");
		std::cout << "\n\n";
	} else {
		std::cout << "\n\n";
		ROS_INFO("Failed to reconfigure parameters.");
		std::cout << "\n\n";
	}
}

void Controller::adaptation(CurrentInfo module_info) {
	float k;

	k = -1*(module_info.getBatteryLevel()/(static_cast<float>(setpoint*module_info.getExpectedBatteryLife())*module_info.getCost()));

	//Here the risk_status comes in to give boundaries to the frequency value (When implemented)
	double new_frequency = module_info.getFrequency() + static_cast<float>(errorCalculation(static_cast<int>(module_info.getExpectedBatteryLife())))*k; //Delta F
	ROS_INFO("Calculated new frequency: %lf", new_frequency);
	if(new_frequency > 0) {
		new_parameters[module_info.getModuleName()] = new_frequency;
	}

	ROS_INFO("Module: %s", module_info.getModuleName().c_str());
	ROS_INFO("New Frequency: %.2f", new_parameters[module_info.getModuleName()]);
}

void Controller::run() {
	ros::Rate loop_rate(adapt_frequency);
	ros::Subscriber receive_data = n.subscribe("controller_info", 1000, &Controller::getInfo, this);
	
	while(ros::ok()) {
		ros::spinOnce();

		if(empty == false) {
			std::map<std::string, CurrentInfo>::iterator info_it;

			std::cout << "\n\n";
			for(info_it = info.begin();info_it != info.end();++info_it) {
				adaptation(info_it->second);
			}
			std::cout << "\n\n";

			//Scheduler service implementation is still missing
			reconfigureParameters(); 

			new_parameters.clear();
			info.clear();
			empty = true;
		}
		
		loop_rate.sleep();
	}
}