#include "component/SchedulableComponent.hpp"

SchedulableComponent::SchedulableComponent(const int32_t &argc, char **argv) : 
	Module(argc, argv), 
	received_name(""), 
	check_frequency(2) {}

SchedulableComponent::~SchedulableComponent() {}

/********************************************************************************
 Description: Basic Module SigInt Handler for module disconnection

	-> Note: Object erasing MUST be done in a tearDown() function
	-> Note2: If module termination is not done via SigINT, this functionality has to be passed to tearDown() function

	@param: int sig - Signal for CTRL+C event
*********************************************************************************/
void SchedulableComponent::schedulingSigIntHandler(int sig) {	

	{	// Unregister from scheduler
		services::SchedulerRegister srv;
		srv.request.name = ros::this_node::getName();
		srv.request.frequency = 0;
		srv.request.deadline = 0;
		srv.request.wce = 0;

		srv.request.connection = false;

		ros::NodeHandle client_handler;

		ros::ServiceClient client_module;

		//Connection to scheduler module management service
		client_module = client_handler.serviceClient<services::SchedulerRegister>("SchedulerRegister");

		if(client_module.call(srv)) {
			ROS_INFO("Succesfully disconnected from scheduler.");
		} else {
			ROS_ERROR("Failed to disconnect from scheduler.");
		}
	}

	{	// Unregister from effector
		services::EffectorRegister srv;
		srv.request.name = ros::this_node::getName();
		srv.request.frequency = 0;
		srv.request.deadline = 0;
		srv.request.wce = 0;

		srv.request.connection = false;

		ros::NodeHandle client_handler;

		ros::ServiceClient client_module;

		//Connection to scheduler module management service
		client_module = client_handler.serviceClient<services::EffectorRegister>("EffectorRegister");

		if(client_module.call(srv)) {
			ROS_INFO("Succesfully disconnected from effector.");
		} else {
			ROS_ERROR("Failed to disconnect from effector.");
		}
	}

	ros::shutdown();
}

void SchedulableComponent::tearDown() {
}

/********************************************************************************
 Description: Basic Module initialization function

 -> MUST be called before run()!!

 @param: None
*********************************************************************************/	
void SchedulableComponent::setUp() {
	//Defining custom SIGINT Handler
	signal(SIGINT, schedulingSigIntHandler);

	{ // Configure module descriptor for scheduling
        double freq, check_frequency;
        int32_t deadline, wce;

        moduleDescriptor.setName(ros::this_node::getName());

        handle.getParam("frequency", freq);
        moduleDescriptor.setFreq(freq);

        handle.getParam("deadline", deadline);
        moduleDescriptor.setDeadline(static_cast<int32_t>(deadline));

        handle.getParam("wce", wce);
        moduleDescriptor.setWorstCaseExecutionTime(static_cast<int32_t>(wce));

        handle.getParam("check_frequency", check_frequency);
        setCheckFrequency(check_frequency);

        moduleDescriptor.setConnection(true);
    }

	{ // register in scheduler
		ros::NodeHandle client_handler;
		client_module = client_handler.serviceClient<services::SchedulerRegister>("SchedulerRegister");

		services::SchedulerRegister srv;
		srv.request.name = moduleDescriptor.getName();
		srv.request.frequency = moduleDescriptor.getFreq();
		srv.request.deadline = moduleDescriptor.getDeadline();
		srv.request.wce = moduleDescriptor.getWorstCaseExecutionTime();
		srv.request.connection = moduleDescriptor.getConnection();

		if(client_module.call(srv)) {
			ROS_INFO("Succesfully connected to scheduler.");
			moduleDescriptor.setConnection(true);
		} else {
			ROS_ERROR("Failed to connect to scheduler.");
		}
	}

	{ // register in effector
		ros::NodeHandle client_handler;
		client_module = client_handler.serviceClient<services::EffectorRegister>("EffectorRegister");

		services::EffectorRegister srv;
		srv.request.name = moduleDescriptor.getName();
		srv.request.frequency = moduleDescriptor.getFreq();
		srv.request.deadline = moduleDescriptor.getDeadline();
		srv.request.wce = moduleDescriptor.getWorstCaseExecutionTime();
		srv.request.connection = moduleDescriptor.getConnection();

		if(client_module.call(srv)) {
			ROS_INFO("Succesfully connected to effector.");
			moduleDescriptor.setConnection(true);
		} else {
			ROS_ERROR("Failed to connect to effector.");
		}
	}

	//Defining module's execution minimum checking period
	period = static_cast<float>(moduleDescriptor.getDeadline())/1000000.0;

	ros::NodeHandle schedule_task_handler, task_finished_handler, notify_active_node;

	//Subscrbing to this module's scheduling topic
	schedule_task = schedule_task_handler.subscribe("effect_" + moduleDescriptor.getName(), 1, &SchedulableComponent::schedulingCallback, this);

	//Publishing in finish topic, which indicates end of module's execution
	task_finished_pub = task_finished_handler.advertise<messages::Event>("collect_event", 1000);
}

/********************************************************************************
 Description: Callback to receive execution command from scheduler

 -> Note: If module receives it's name, execution is allowed

 @param: const messages::ReconfigurationCommand& msg - name received
*********************************************************************************/
void SchedulableComponent::schedulingCallback(const messages::ReconfigurationCommand& msg) {
	ROS_INFO("I heard: [%s]", msg.target.c_str());
	received_name = msg.target;

	if(received_name != moduleDescriptor.getName()) return;

	body();

	messages::Event f_msg;

	f_msg.source = moduleDescriptor.getName();
	f_msg.type = "finished";

	task_finished_pub.publish(f_msg);
}

/********************************************************************************
 Description: Basic module run function

 -> Note: Module functionality goes inside if

 @param: None
*********************************************************************************/
void SchedulableComponent::run() {

	SchedulableComponent::setUp();
	setUp();

	//Defining checking frequency
	//Note: By default it is 2/period
	//Note2: The bigger check_frequency is, higher the granularity
	ros::Rate loop_rate((1/period)*check_frequency);

	while(!ros::isShuttingDown()) {
		//receive name
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	tearDown();
	SchedulableComponent::tearDown();
}

void SchedulableComponent::setReceivedName(const std::string &received_name) {
	this->received_name = received_name;
}

std::string SchedulableComponent::getReceivedName() const {
	return this->received_name;
}

void SchedulableComponent::setCheckFrequency(const double &check_frequency) {
	this->check_frequency = check_frequency;
}

double SchedulableComponent::getCheckFrequency() const {
	return this->check_frequency;
}

void SchedulableComponent::setPeriod(const double &period) {
	this->period = period;
}

double SchedulableComponent::getPeriod() const {
	return this->period;
}