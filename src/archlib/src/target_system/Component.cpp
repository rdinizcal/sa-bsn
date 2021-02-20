#include "archlib/target_system/Component.hpp"

namespace arch {
	namespace target_system {
		Component::Component(int &argc, char **argv, const std::string &name) : ROSComponent(argc, argv, name) {}

		Component::~Component() {}

		void Component::setUp() {
			double freq = 1;
			// handle.getParam("frequency", freq);
			ROS_INFO("Module initial frequency: %lf", freq);
			rosComponentDescriptor.setFreq(freq);

			collect_event = handle.advertise<archlib::Event>("collect_event", 10);
			while(collect_event.getNumSubscribers() < 1) {} // to cope with the delay on opening the connection

			collect_status = handle.advertise<archlib::Status>("collect_status", 10);
			while(collect_status.getNumSubscribers() < 1) {}

			collect_energy_status = handle.advertise<archlib::EnergyStatus>("collect_energy_status", 10);
			while(collect_energy_status.getNumSubscribers() < 1) {}

			sendStatus("init");
			activate();

			signal(SIGINT, sigIntHandler);

			// register in effector
			ros::NodeHandle client_handler;
            ros::ServiceClient client_module;

			client_module = client_handler.serviceClient<archlib::EffectorRegister>("EffectorRegister");

			archlib::EffectorRegister srv;

			srv.request.name = getRosNodeName(ros::this_node::getName(), ros::this_node::getNamespace());
			srv.request.connection = true;

			if(client_module.call(srv)) {
				ROS_INFO("Succesfully connected to effector.");
			} else {
				ROS_ERROR("Failed to connect to effector.");
			}
			
		}

		void Component::tearDown() {
			/* sigIntHandler is making the job of the tearDown
			sendStatus("finish");
			deactivate();
			*/
		}

		void Component::sigIntHandler(int signal) {
			shutdownComponent();
		}

		void Component::shutdownComponent() {
			archlib::Event eventMsg;
			archlib::Status statusMsg;

			eventMsg.source = getRosNodeName(ros::this_node::getName(), ros::this_node::getNamespace());
			eventMsg.content = "deactivate";

			statusMsg.source = getRosNodeName(ros::this_node::getName(), ros::this_node::getNamespace());
			statusMsg.content = "status";

			ros::NodeHandle nh;

			ros::Publisher last_event = nh.advertise<archlib::Event>("collect_event", 10);
			while(last_event.getNumSubscribers() < 1) {} // to cope with the delay on opening the connection
			last_event.publish(eventMsg);

			ros::Publisher last_status = nh.advertise<archlib::Status>("collect_status", 10);
			while(last_status.getNumSubscribers() < 1) {}
			last_status.publish(statusMsg);

			// Unregister from effector
			archlib::EffectorRegister srv;
			srv.request.name = getRosNodeName(ros::this_node::getName(), ros::this_node::getNamespace());
			srv.request.connection = false;

			ros::NodeHandle client_handler;
            ros::ServiceClient client_module;

			//Connection to scheduler module management service
			client_module = client_handler.serviceClient<archlib::EffectorRegister>("EffectorRegister");

			if(client_module.call(srv)) {
				ROS_INFO("Succesfully disconnected from effector.");
			} else {
				ROS_ERROR("Failed to disconnect from effector.");
			}
		
			ros::shutdown();
		}

		int32_t Component::run() {
			setUp();

			while(ros::ok()) {
				ros::Rate loop_rate(rosComponentDescriptor.getFreq());
				ros::spinOnce();

				sendStatus("running");
				try{
					body();
					sendStatus("success");
				} catch (const std::exception& e) {
					sendStatus("fail");
				} 
				loop_rate.sleep();
			}
			
			tearDown();
			return 0;
		}

		void Component::sendEvent(const std::string &content) {
			archlib::Event msg;

			msg.source = rosComponentDescriptor.getName();
			msg.content = content;
			msg.freq = rosComponentDescriptor.getFreq();

			collect_event.publish(msg);
		}

		void Component::sendStatus(const std::string &content) {
			archlib::Status msg;

			msg.source = rosComponentDescriptor.getName();
			msg.content = content;

			collect_status.publish(msg);
		}

		void Component::sendEnergyStatus(const double &cost) {
			archlib::EnergyStatus msg;

			msg.source = rosComponentDescriptor.getName();
			msg.content = std::to_string(cost);

			collect_energy_status.publish(msg);
		}

		void Component::reconfigure(const archlib::AdaptationCommand::ConstPtr& msg) {
			reconfigure(msg);
		}

		void Component::activate() {
			sendEvent("activate");
			status = true;
		}

        void Component::deactivate() {
			sendEvent("deactivate");
			status = false;
		}
	}
}
