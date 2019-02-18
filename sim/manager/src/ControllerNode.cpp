#include "ControllerNode.hpp"

ControllerNode::ControllerNode(int  &argc, char **argv, std::string name) {
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, name);
}

ControllerNode::~ControllerNode() {}

void ControllerNode::setUp() {}

void ControllerNode::tearDown() {}


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 * I suppose one can declare several chatterCallbacks, each one for each type of
 * message. Cool isn't it ? It should be especially util for receiving distinct
 * tasks and contexts.
 */
void ControllerNode::receiveTaskInfo(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void ControllerNode::receiveContextInfo(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void ControllerNode::run(){

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle publisher_handler;
    ros::NodeHandle subscriber_handler;

    /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.  advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
	ros::Publisher actuator_pub = publisher_handler.advertise<std_msgs::String>("manager_actuator", 1000);

    /** **********************************************
     *                      MONITOR 
    /* ***********************************************
     * receive task (id, cost, reliability)
     *      update list of tasks
     * receive context (id, bool)
     *      reset setpoints (cost and reliability)
     * ***********************************************
     */ 
    /**
     * The subscribe() call is how you tell ROS that you want to receive messages
     * on a given topic.  This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing.  Messages are passed to a callback function, here
     * called chatterCallback.  subscribe() returns a Subscriber object that you
     * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
     * object go out of scope, this callback will automatically be unsubscribed from
     * this topic.
     *
     * The second parameter to the subscribe() function is the size of the message
     * queue.  If messages are arriving faster than they are being processed, this
     * is the number of messages that will be buffered up before beginning to throw
     * away the oldest ones.
     */
    ros::Subscriber sensor_task_sub = subscriber_handler.subscribe("manager_sensor", 1000, receiveTaskInfo);
    ros::Subscriber sensor_context_sub = subscriber_handler.subscribe("manager_sensor", 1000, receiveContextInfo);

	ros::Rate loop_rate(10); // execution frequency f = 10 Hz

    while (ros::ok()){

        loop_rate.sleep();

        /**
        /** ***********************************************
         *                      ANALYZE
        /* ***********************************************
         * analyze whether the setpoints have been violat-
         * ed
         *      plug in formulae and evaluate current cost
         *           and reliabiliy
         *      compare current values with tresholds
         * ***********************************************
        /** ***********************************************
         *                       PLAN
        /* ***********************************************
         * exhaustively analyze whether an action fits the
         *  setpoints
         * ***********************************************
        /** ***********************************************
         *                      EXECUTE
        /* ***********************************************
         * if so, send messages containing the actions to 
         *    the modules
         * ***********************************************
         */

    }

    return;
}
