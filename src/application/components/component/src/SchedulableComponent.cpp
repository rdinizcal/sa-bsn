#include "component/SchedulableComponent.hpp"

SchedulableComponent::SchedulableComponent(const int32_t &argc, char **argv) : 
	Module(argc, argv), 
	received_name(""), 
	topic_name(""), 
	check_frequency(2) {}

SchedulableComponent::~SchedulableComponent() {}

void SchedulableComponent::schedulingSigIntHandler(int sig) {	
	/********************************************************************************
	Description: Basic Module SigInt Handler for module disconnection

	-> Note: Object erasing MUST be done in a tearDown() function
	-> Note2: If module termination is not done via SigINT, this functionality has to be
	passed to tearDown() function

	@param: int sig - Signal for CTRL+C event
	*********************************************************************************/

	//Setting request parameters with connection = false
	services::SchedulerServerData srv;
	srv.request.name = ros::this_node::getName();
	srv.request.frequency = 0;
	srv.request.deadline = 0;
	srv.request.wce = 0;

	srv.request.connection = false;

	ros::NodeHandle client_handler, param_handler("~");

	ros::ServiceClient client_module;

	//Connection to scheduler module management service
	client_module = client_handler.serviceClient<services::SchedulerServerData>("ModuleManagement");

	if(client_module.call(srv)) {
		ROS_INFO("Succesfully disconnected from scheduler.");
	} else {
		ROS_ERROR("Failed to disconnect from scheduler.");
	}

	ros::shutdown();
}

void SchedulableComponent::tearDown() {
	ROS_INFO("SchedulableComponent::tearDown()");
}

/********************************************************************************
 Description: Basic Module initialization function

-> MUST be called before run()!!

@param: None
*********************************************************************************/	
void SchedulableComponent::setUp() {
	ROS_INFO("SchedulableComponent::setUp()");

	//Defining custom SIGINT Handler
	signal(SIGINT, schedulingSigIntHandler);

	ros::NodeHandle client_handler, param_handler("~");
	client_module = client_handler.serviceClient<services::SchedulerServerData>("ModuleManagement");

	services::SchedulerServerData srv;
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

	//Defining module's execution minimum checking period
	period = static_cast<float>(moduleDescriptor.getDeadline())/1000000.0;

	ros::NodeHandle schedule_task_handler, task_finished_handler;

	//Subscrbing to this module's scheduling topic
	//topic_name = ros::this_node::getName().substr(1) + "topic";
	//sched = scheduling_handler.subscribe(topic_name, 5, &SchedulableComponent::schedulingCallback, this);
	schedule_task = schedule_task_handler.subscribe("schedule_task", 5, &SchedulableComponent::schedulingCallback, this);

	//Publishing in finish topic, which indicates end of module's execution
	//finish_topic_name = topic_name + "_finish";
	//scheduling_pub = finish_scheduling_handler.advertise<messages::Finish>(finish_topic_name = topic_name + "_finish", 1);
	task_finished_pub = task_finished_handler.advertise<messages::Finish>("task_finished", 1);

}

/********************************************************************************
 Description: Callback to receive execution command from scheduler

 -> Note: If module receives it's name, execution is allowed

 @param: const messages::Init& msg - name received
*********************************************************************************/
void SchedulableComponent::schedulingCallback(const messages::Init& msg) {
	ROS_INFO("I heard: [%s]", msg.module_name.c_str());
	received_name = msg.module_name;

	body();

	ros::Time finish_time = ros::Time::now();

	messages::Finish f_msg;

	f_msg.module_name= ros::this_node::getName();
	f_msg.sec = finish_time.sec;
	f_msg.nsec = finish_time.nsec;

	//Publishing in scheduling_finish topic
	task_finished_pub.publish(f_msg);
	ros::spinOnce();
	

}

/********************************************************************************
 Description: Basic module run function

 -> Note: Module functionality goes inside if

 @param: None
*********************************************************************************/
void SchedulableComponent::run() {

	setUp();
	SchedulableComponent::setUp();

	//Defining checking frequency
	//Note: By default it is 2/period
	//Note2: The bigger check_frequency is, higher the granularity
	ros::Rate loop_rate((1/period)*check_frequency);

	while(!ros::isShuttingDown()) {
		//receive name
		ros::spinOnce();
		loop_rate.sleep();
	}
	/* 
		if(received_name == moduleDescriptor.getName()) {
			//Basic printing, can be uncommented if desirable
			ROS_INFO("Running");

			body();

			ros::Time finish_time = ros::Time::now();

			messages::Finish msg;

			msg.name = ros::this_node::getName();
			msg.sec = finish_time.sec;
			msg.nsec = finish_time.nsec;

			//Publishing in scheduling_finish topic
			scheduling_pub.publish(msg);

			ros::spinOnce();

			//Sleep until next check
			loop_rate.sleep();
		} else {
			//Basic printing, can be uncommented if desirable
			ROS_INFO("Not Running");

			//Sleep until next check
			loop_rate.sleep();
		}

		received_name = "";
	}
	*/

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