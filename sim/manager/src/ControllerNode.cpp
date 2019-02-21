#include "ControllerNode.hpp"

using namespace bsn::goalmodel;

ControllerNode::ControllerNode(int  &argc, char **argv, std::string name):
    tasks(),
    contexts(),

    cost_expression(),
    reliability_expression(),

    cost_formula_reliabilities(),
    cost_formula_frequencies(),
    cost_formula_costs(),
    cost_formula_contexts(),

    reliability_formula_reliabilities(),
    reliability_formula_frequencies(),
    reliability_formula_contexts(),

    actions(),

    cost_setpoint(0.53),
    reliability_setpoint(0.90) {
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

void ControllerNode::setUp() {
    /*
    { // Set up map {id,object} of leaf task from goal model
        // Pulse oximeter
        tasks.insert(std::pair<std::string,Task>("G3_T1.11",Task("G3_T1.11","Read data","W_G3_T1_11","R_G3_T1_11","F_G3_T1_11")));
        tasks.insert(std::pair<std::string,Task>("G3_T1.12",Task("G3_T1.12","Filter data","W_G3_T1_12","R_G3_T1_12","F_G3_T1_12")));
        tasks.insert(std::pair<std::string,Task>("G3_T1.13",Task("G3_T1.13","Transfer data","W_G3_T1_13","R_G3_T1_13","F_G3_T1_13")));

        // Electrocardiograph
        tasks.insert(std::pair<std::string,Task>("G3_T1.21",Task("G3_T1.21","Read data","W_G3_T1_21","R_G3_T1_21","F_G3_T1_21")));
        tasks.insert(std::pair<std::string,Task>("G3_T1.22",Task("G3_T1.22","Filter data","W_G3_T1_22","R_G3_T1_22","F_G3_T1_22")));
        tasks.insert(std::pair<std::string,Task>("G3_T1.23",Task("G3_T1.23","Transfer data","W_G3_T1_23","R_G3_T1_23","F_G3_T1_23")));

        // Thermometer
        tasks.insert(std::pair<std::string,Task>("G3_T1.31",Task("G3_T1.31","Read data","W_G3_T1_31","R_G3_T1_31","F_G3_T1_31")));
        tasks.insert(std::pair<std::string,Task>("G3_T1.32",Task("G3_T1.32","Filter data","W_G3_T1_32","R_G3_T1_32","F_G3_T1_32")));
        tasks.insert(std::pair<std::string,Task>("G3_T1.33",Task("G3_T1.33","Transfer data","W_G3_T1_33","R_G3_T1_33","F_G3_T1_33")));

        // Sphygmomanometer
        tasks.insert(std::pair<std::string,Task>("G3_T1.411",Task("G3_T1.411","Read diastolic","W_G3_T1_411","R_G3_T1_411","F_G3_T1_411")));
        tasks.insert(std::pair<std::string,Task>("G3_T1.412",Task("G3_T1.412","Read systolic","W_G3_T1_412","R_G3_T1_412","F_G3_T1_412")));
        tasks.insert(std::pair<std::string,Task>("G3_T1.42",Task("G3_T1.42","Filter data","W_G3_T1_42","R_G3_T1_42","F_G3_T1_42")));
        tasks.insert(std::pair<std::string,Task>("G3_T1.43",Task("G3_T1.43","Transfer data","W_G3_T1_43","R_G3_T1_43","F_G3_T1_43")));

        // Centralhub
        tasks.insert(std::pair<std::string,Task>("G4_T1.1",Task("G4_T1.1","Fuse sensor data","W_G4_T1_1","R_G4_T1_1","F_G4_T1_1")));
        tasks.insert(std::pair<std::string,Task>("G4_T1.2",Task("G4_T1.2","Detect patient status","W_G4_T1_2","R_G4_T1_2","F_G4_T1_2")));
        tasks.insert(std::pair<std::string,Task>("G4_T1.3",Task("G4_T1.3","Persist data","W_G4_T1_3","R_G4_T1_3","F_G4_T1_3")));
    }

    { // Set up map {id,object} of context from goal model
        contexts.insert(std::pair<std::string,Context>("SaO2_available",Context("SaO2_available",false,"CTX_G3_T1_1")));
        contexts.insert(std::pair<std::string,Context>("ECG_available",Context("ECG_available",false,"CTX_G3_T1_2")));
        contexts.insert(std::pair<std::string,Context>("TEMP_available",Context("TEMP_available",false,"CTX_G3_T1_3")));
        contexts.insert(std::pair<std::string,Context>("ABP_available",Context("ABP_available",false,"CTX_G3_T1_4")));
    }

    { // Set up cost and reliability expressions
        std::ifstream cost_file;
        std::string cost_formula;

        std::ifstream reliability_file;
        std::string reliability_formula;

        try{
            cost_file.open("../formulae/cost.formula");
            std::getline(cost_file,cost_formula);
            cost_file.close();
        } catch (std::ifstream::failure e) { std::cerr << "Exception opening/reading/closing file (cost.formula)\n"; }

        try{
            reliability_file.open("../formulae/reliability.formula");
            std::getline(reliability_file,reliability_formula);
            reliability_file.close();
        } catch (std::ifstream::failure e) { std::cerr << "Exception opening/reading/closing file (reliability.formula)\n"; }
    
        cost_expression = Lepton::Parser::parse(cost_formula).createCompiledExpression();
        reliability_expression = Lepton::Parser::parse(reliability_formula).createCompiledExpression();

        for (std::pair<std::string,Task> task : tasks){
            cost_formula_reliabilities          .insert(std::pair<std::string,double&>(task.second.getTask(),cost_expression.getVariableReference(task.second.getReliabilitySymbol())));
            cost_formula_frequencies            .insert(std::pair<std::string,double&>(task.second.getTask(),cost_expression.getVariableReference(task.second.getFrequencySymbol())));
            cost_formula_costs                  .insert(std::pair<std::string,double&>(task.second.getTask(),cost_expression.getVariableReference(task.second.getCostSymbol())));
            
            reliability_formula_reliabilities   .insert(std::pair<std::string,double&>(task.second.getTask(),reliability_expression.getVariableReference(task.second.getReliabilitySymbol())));
            reliability_formula_frequencies     .insert(std::pair<std::string,double&>(task.second.getTask(),reliability_expression.getVariableReference(task.second.getFrequencySymbol())));
        }

        
        for (std::pair<std::string,Context> context : contexts) {
            cost_formula_contexts               .insert(std::pair<std::string,double&>(context.second.getContext(),cost_expression.getVariableReference(context.second.getContextSymbol())));
            reliability_formula_contexts        .insert(std::pair<std::string,double&>(context.second.getContext(),reliability_expression.getVariableReference(context.second.getContextSymbol())));
        }
    }

    { // Set up actions
        
        actions = std::vector<std::vector<double>> {
                                        {0.8,0.85,0.9,0.95,1},
                                        {0.8,0.85,0.9,0.95,1},
                                        {0.8,0.85,0.9,0.95,1},
                                        {0.8,0.85,0.9,0.95,1}};
        
        
        
        for (int idx = 0, v = 0, w = 0, x = 0, y = 0, z = 0; idx < std::pow(5,7); ++idx){
            actions.push_back({(double)v, (double)w, (double)x, (double)y, (double)z});

            if(++z == 8) { 
                z = 0;
                if(++y == 8) { 
                    y = 0;
                    if(++x == 8) { 
                        x = 0;
                        if(++w == 8) { 
                            w = 0;
                            if(++v == 8) { 
                                v = 0;
                            }
                        }
                    }
                }
            }
        }

        for (std::vector<std::vector<double>>::iterator it = actions.begin(); it != actions.end(); ++it) {
            for (std::vector<double>::iterator itt = (*it).begin(); itt != (*it).end(); ++itt) {
                if ((int)*itt == 0) *itt = 0.775;
                else if ((int)*itt == 1) *itt = 0.8125;
                else if ((int)*itt == 2) *itt = 0.85;
                else if ((int)*itt == 3) *itt = 0.8875;
                else if ((int)*itt == 4) *itt = 0.925;
                else if ((int)*itt == 5) *itt = 0.9625;
                else if ((int)*itt == 6) *itt = 1;
            }
        }

        for (double idx = 0; idx <= 1; idx += 0.0009765625) actions.push_back({idx});
    }*/
}


/** **************************************************************
 *                          MONITOR 
/* ***************************************************************
 * receive task (id, cost, reliability) update list of tasks
 * receive context (id, bool) reset setpoints (cost and reliability)
 * ***************************************************************
*/ 
/**
 * This tutorial demonstrates simple receipt of messages over the 
 * ROS system. I suppose one can declare several chatterCallbacks, 
 * each one for each type of message. Cool isn't it ? It should be
 * especially util for receiving distinct tasks and contexts.
*/
void ControllerNode::receiveTaskInfo(const bsn::TaskInfo::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->task_id.c_str());

    std::string id = msg->task_id;

    //tasks[id].setCost(msg->cost);
    //tasks[id].setReliability(msg->reliability);
    //tasks[id].setFrequency(msg->frequency);

    analyze();
}

void ControllerNode::receiveContextInfo(const bsn::ContextInfo::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->context_id.c_str());

    std::string id = msg->context_id;

    //contexts[id].setValue(msg->status.data);

    analyze();
}

/** **************************************************************
 *                         ANALYZE
/* ***************************************************************
 * analyze whether the setpoints have been violated
 *      -plug in formulae and evaluate current cost and reliabiliy   
 *      -compare current values with tresholds
 * ***************************************************************
*/ 
void ControllerNode::analyze() {
    /*double reliability;
    double cost;

    { // plug in costs, reliabilities, frequencies and contexts and evaluate formulas
        { // in cost formula
            for (std::pair<std::string,double&> cost_formula_reliability : cost_formula_reliabilities) {
                cost_formula_reliability.second = tasks[cost_formula_reliability.first].getReliability();
                //std::cout << tasks[cost_formula_reliability.first].getReliabilitySymbol() << ": " << cost_formula_reliability.second << std::endl;
            }

            for (std::pair<std::string,double&> cost_formula_frequency : cost_formula_frequencies) {
                cost_formula_frequency.second = tasks[cost_formula_frequency.first].getFrequency();
                //std::cout << tasks[cost_formula_frequency.first].getFrequencySymbol() << ": " << cost_formula_frequency.second << std::endl;
            }

            for (std::pair<std::string,double&> cost_formula_cost : cost_formula_costs) {
                cost_formula_cost.second = tasks[cost_formula_cost.first].getCost();
                //std::cout << tasks[cost_formula_cost.first].getCostSymbol() << ": " << cost_formula_cost.second << std::endl;
            }

            for (std::pair<std::string,double&> cost_formula_context : cost_formula_contexts) {
                cost_formula_context.second = contexts[cost_formula_context.first].getValue() ? 1:0;
                //std::cout << contexts[cost_formula_context.first].getContextSymbol() << ": " << cost_formula_context.second << std::endl;
            }

            cost = cost_expression.evaluate();
        }

        { // in reliability formula
            for (std::pair<std::string,double&> reliability_formula_reliability : reliability_formula_reliabilities) {
                reliability_formula_reliability.second = tasks[reliability_formula_reliability.first].getReliability();
            }

            for (std::pair<std::string,double&> reliability_formula_frequency : reliability_formula_frequencies) {
                reliability_formula_frequency.second = tasks[reliability_formula_frequency.first].getFrequency();
            }

            for (std::pair<std::string,double&> reliability_formula_context : reliability_formula_contexts) {
                reliability_formula_context.second = contexts[reliability_formula_context.first].getValue() ? 1:0;
            }

            reliability = reliability_expression.evaluate();
        }
    }

    if (reliability < reliability_setpoint*0.95 || reliability > reliability_setpoint*1.05 || 
        cost < cost_setpoint*0.95 || cost > cost_setpoint*1.05) {
        
        plan();

    }*/
}

/** **************************************************************
 *                          PLAN
/* ***************************************************************
 * exhaustively analyze whether an action fits the setpoints
 * ***************************************************************
*/
void ControllerNode::plan() {
    /*std::map<std::vector<double>, std::vector<double>> policies;
    double cost;
    double reliability;

    for (std::vector<double> action : actions ) { // substitutues each action the formulas and calculates cost and reliability
        { // in cost formula
            for (std::pair<std::string,double&> cost_formula_frequency : cost_formula_frequencies) {

                if (cost_formula_frequency.first.find("G3_T1.1") != std::string::npos) {
                    cost_formula_frequency.second = action[0];
                } else if (cost_formula_frequency.first.find("G3_T1.2") != std::string::npos) {
                    cost_formula_frequency.second = action[1];
                } else if (cost_formula_frequency.first.find("G3_T1.3") != std::string::npos) {
                    cost_formula_frequency.second = action[2];
                } else if (cost_formula_frequency.first.find("G3_T1.4") != std::string::npos) {
                    cost_formula_frequency.second = action[3];
                } else if (cost_formula_frequency.first.find("G4_T1") != std::string::npos) {
                    cost_formula_frequency.second = action[0];
                }
                
            }

            cost = cost_expression.evaluate();
        }

        { // in reliability formula
            for (std::pair<std::string,double&> reliability_formula_frequency : reliability_formula_frequencies) {
                if (reliability_formula_frequency.first.find("G3_T1.1") != std::string::npos) {
                    reliability_formula_frequency.second = action[0];
                } else if (reliability_formula_frequency.first.find("G3_T1.2") != std::string::npos) {
                    reliability_formula_frequency.second = action[1];
                } else if (reliability_formula_frequency.first.find("G3_T1.3") != std::string::npos) {
                    reliability_formula_frequency.second = action[2];
                } else if (reliability_formula_frequency.first.find("G3_T1.4") != std::string::npos) {
                    reliability_formula_frequency.second = action[3];
                } else  if (reliability_formula_frequency.first.find("G4_T1") != std::string::npos) {
                    reliability_formula_frequency.second = action[0];
                }
            }

            reliability = reliability_expression.evaluate();
        }
        
        policies[action] = {reliability,cost};
    }

    for (std::pair<std::vector<double>,std::vector<double>> policy : policies) {

        std::cout << "[" << policy.first[0] /*<< "," << policy.first[1] << "," << policy.first[2] << "," << policy.first[3] << ", " << policy.first[4] << << "] ";
        std::cout << "--> reliability: " << policy.second[0] << " cost: " << policy.second[1] << std::endl;

        if(policy.second[0] >= reliability_setpoint*0.98 && policy.second[0] <= reliability_setpoint*1.05 &&
            policy.second[1] >= cost_setpoint*0.98 && policy.second[1] <= cost_setpoint*1.05 ) {
            std::cout << "Adaptation triggered!" << std::endl;
            
            execute();

            break;
        }
    }*/
}

/** **************************************************************
 *                         EXECUTE
/* ***************************************************************
 * if so, send messages containing the actions to the modules
 * ***************************************************************
*/
void ControllerNode::execute() {
    
    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    //ros::NodeHandle publisher_handler;

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
	//ros::Publisher actuator_pub = publisher_handler.advertise<bsn::String>("manager_actuator", 1000);
}

void ControllerNode::run(){

    ros::NodeHandle n;
   
    ros::Subscriber t_sub = n.subscribe("manager_sensor", 1000, &ControllerNode::receiveTaskInfo, this);
    ros::Subscriber c_sub = n.subscribe("manager_sensor", 1000, &ControllerNode::receiveContextInfo, this);

    ros::spin();


    return;
}
