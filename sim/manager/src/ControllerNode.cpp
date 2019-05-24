#include "ControllerNode.hpp"
#include "operation/Operation.hpp"

#include <memory>
#include <map>
using namespace bsn::goalmodel;

ControllerNode::ControllerNode(int  &argc, char **argv, std::string name):
    tasks(),
    contexts(),

    cost_expression(),
    reliability_expression()

    ros::init(argc, argv, name);
}

void ControllerNode::setTaskValue(std::string id, double value) {
    this->tasks[id] = value;
}

double ControllerNode::getTaskValue(std::string id) {
    return this->tasks[id];
}

bool ControllerNode::isCost(std::string id) {
    return id[0] == ('W');
}

ControllerNode::~ControllerNode() {}

void ControllerNode::setUp() {

    GoalTree goalModel("Body Sensor Network");

    { // Set up the goal tree goalModel        
        LeafTask g3_t1_11("G3_T1.11","Read data", Property("W_G3_T1_11", 1),Property("R_G3_T1_11", 1),Property("F_G3_T1_11",1));
        LeafTask g3_t1_12("G3_T1.12","Filter data", Property("W_G3_T1_12", 1),Property("R_G3_T1_12", 1),Property("F_G3_T1_12",1));
        LeafTask g3_t1_13("G3_T1.13","Transfer data", Property("W_G3_T1_13", 1),Property("R_G3_T1_13", 1),Property("F_G3_T1_13",1));

        Task g3_t1_1("G3_T1.1", "Collect SaO2 data");
        g3_t1_1.addChild(std::make_shared<LeafTask>(g3_t1_11));
        g3_t1_1.addChild(std::make_shared<LeafTask>(g3_t1_12));
        g3_t1_1.addChild(std::make_shared<LeafTask>(g3_t1_13));

        LeafTask g3_t1_21("G3_T1.21","Read data", Property("W_G3_T1_21", 1),Property("R_G3_T1_21", 1),Property("F_G3_T1_21",1));
        LeafTask g3_t1_22("G3_T1.22","Filter data", Property("W_G3_T1_22", 1),Property("R_G3_T1_22", 1),Property("F_G3_T1_22",1));
        LeafTask g3_t1_23("G3_T1.23","Transfer data", Property("W_G3_T1_23", 1),Property("R_G3_T1_23", 1),Property("F_G3_T1_23",1));

        Task g3_t1_2("G3_T1.2", "Collect ECG data");
        g3_t1_2.addChild(std::make_shared<LeafTask>(g3_t1_21));
        g3_t1_2.addChild(std::make_shared<LeafTask>(g3_t1_22));
        g3_t1_2.addChild(std::make_shared<LeafTask>(g3_t1_23));

        LeafTask g3_t1_31("G3_T1.31","Read data", Property("W_G3_T1_31", 1),Property("R_G3_T1_31", 1),Property("F_G3_T1_31",1));
        LeafTask g3_t1_32("G3_T1.32","Filter data", Property("W_G3_T1_32", 1),Property("R_G3_T1_32", 1),Property("F_G3_T1_32",1));
        LeafTask g3_t1_33("G3_T1.33","Transfer data", Property("W_G3_T1_33", 1),Property("R_G3_T1_33", 1),Property("F_G3_T1_33",1));

        Task g3_t1_3("G3_T1.3", "Collect TEMP data");
        g3_t1_3.addChild(std::make_shared<LeafTask>(g3_t1_31));
        g3_t1_3.addChild(std::make_shared<LeafTask>(g3_t1_32));
        g3_t1_3.addChild(std::make_shared<LeafTask>(g3_t1_33));

        LeafTask g3_t1_411("G3_T1.411","Read diastolic", Property("W_G3_T1_411", 1),Property("R_G3_T1_411", 1),Property("F_G3_T1_411",1));
        LeafTask g3_t1_412("G3_T1.412","Read systolic", Property("W_G3_T1_412", 1),Property("R_G3_T1_412", 1),Property("F_G3_T1_412",1));
        LeafTask g3_t1_42("G3_T1.42","Filter data", Property("W_G3_T1_42", 1),Property("R_G3_T1_42", 1),Property("F_G3_T1_42",1));
        LeafTask g3_t1_43("G3_T1.43","Transfer data", Property("W_G3_T1_43", 1),Property("R_G3_T1_43", 1),Property("F_G3_T1_43",1));

        Task g3_t1_4("G3_T1.4", "Collect ABP data");
        g3_t1_4.addChild(std::make_shared<LeafTask>(g3_t1_411));
        g3_t1_4.addChild(std::make_shared<LeafTask>(g3_t1_412));
        g3_t1_4.addChild(std::make_shared<LeafTask>(g3_t1_42));
        g3_t1_4.addChild(std::make_shared<LeafTask>(g3_t1_43));
 
        Task g3_t1("G3_T1", "Monitor vital signs");
        g3_t1.addChild(std::make_shared<Task>(g3_t1_1));
 
        g3_t1.addChild(std::make_shared<Task>(g3_t1_2));
        g3_t1.addChild(std::make_shared<Task>(g3_t1_3));
        g3_t1.addChild(std::make_shared<Task>(g3_t1_4));

        Goal g3("G3", "Vital signs are monitored");
        g3.addChild(std::make_shared<Task>(g3_t1));

        LeafTask g4_t1_1("G4_T1.1","Read data", Property("W_G4_T1_1", 1),Property("R_G4_T1_1", 1),Property("F_G4_T1_1",1));
        LeafTask g4_t1_2("G4_T1.2","Filter data", Property("W_G4_T1_2", 1),Property("R_G4_T1_2", 1),Property("F_G4_T1_3",1));
        LeafTask g4_t1_3("G4_T1.3","Transfer data", Property("W_G4_T1_3", 1),Property("R_G4_T1_3", 1),Property("F_G4_T1_3",1));

        Task g4_t1("G4_T1", "Analyze vital signs");
        g4_t1.addChild(std::make_shared<LeafTask> (g4_t1_1));
        g4_t1.addChild(std::make_shared<LeafTask> (g4_t1_2));
        g4_t1.addChild(std::make_shared<LeafTask> (g4_t1_3));

        Goal g4("G4", "Vital signs are analyzed");
        g4.addChild(std::make_shared<Task>(g4_t1));

        Goal g2("G2", "Patient status is monitored");
        g2.addChild(std::make_shared<Goal>(g4));

        Goal g1("G1", "Emergency is detected");
        g1.addChild(std::make_shared<Goal>(g2));

        goalModel.addRootGoal(std::make_shared<Goal>(g1));
    }

    /*
    { // Set up map {id,object} of context from goal goalModel
        contexts.insert(std::pair<std::string,Context>("SaO2_available",Context("SaO2_available",false,"CTX_G3_T1_1")));
        contexts.insert(std::pair<std::string,Context>("ECG_available",Context("ECG_available",false,"CTX_G3_T1_2")));
        contexts.insert(std::pair<std::string,Context>("TEMP_available",Context("TEMP_available",false,"CTX_G3_T1_3")));
        contexts.insert(std::pair<std::string,Context>("ABP_available",Context("ABP_available",false,"CTX_G3_T1_4")));
    }
    */

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
    
        cost_expression = bsn::model::Formula(cost_formula);
        reliability_expression = bsn::model::Formula(reliability_formula);

        std::vector<std::shared_ptr<bsn::goalmodel::LeafTask>> leafTasks = goalModel.getLeafTasks();

        //set initial conditions: order should be respected

        //reli expr
        for(std::vector<std::shared_ptr<bsn::goalmodel::LeafTask>>::iterator it = leafTasks.begin();
                it != leafTasks.end(); it++) {
            setTaskValue((*it)->getReliability().getID(), (*it)->getReliability().getValue());
            setTaskValue((*it)->getFrequency().getID(), (*it)->getFrequency().getValue());
        }

        for(std::map<std::string, double>::const_iterator it = tasks.begin();
                it != tasks.end(); it++) {
            props.push_back(it->first);
            values.push_back(it->second);
        }

        this->reli_value = reliability_expression.apply(props, values);

        //cost expr
        for(std::vector<std::shared_ptr<bsn::goalmodel::LeafTask>>::iterator it = leafTasks.begin();
                it != leafTasks.end(); it++) {
            setTaskValue((*it)->getCost().getID(), (*it)->getCost().getValue());
        }        

        // Checks if given variable is a cost in order to avoid adding extras and
        // unnecessarily clearing the vector

        for(std::map<std::string, double>::const_iterator it = tasks.begin();
                it != tasks.end(); it++) {
            if(isCost(it->first)) {
                props.push_back(it->first);
                values.push_back(it->second);
            }
        }

        this->cost_value = cost_expression.apply(props, values);

}


/** **************************************************************
 *                          MONITOR 
/* ***************************************************************
 * receive task (id, cost, reliability) update list of tasks
 * receive context (id, bool) reset setpoints (cost and reliability)
 * ***************************************************************
*/ 
void ControllerNode::receiveTaskInfo(const bsn::TaskInfo::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->task_id.c_str());

    std::string id = msg->task_id;
    std::string id_aux = id;
    std::replace(id.begin(), id.end(), '.', '_');
    std::string costID = "W_" + id;
    std::string reliabilityID = "R_" + id;
    std::string frequencyID = "F_" + id;

    setTaskValue(costID, msg->cost);
    setTaskValue(reliabilityID, msg->reliability);
    setTaskValue(frequencyID, msg->frequency);
    
    analyze(id_aux);
}

void ControllerNode::receiveContextInfo(const bsn::ContextInfo::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->context_id.c_str());

    std::string id = msg->context_id;

    //contexts[id].setValue(msg->status.data);

//    analyze();
}

/** **************************************************************
 *                         ANALYZE
/* ***************************************************************
 * analyze whether the setpoints have been violated
 *      -plug in formulae and evaluate current cost and reliabiliy   
 *      -compare current values with tresholds
 * ***************************************************************
*/ 
void ControllerNode::analyze(std::string id) {
    
    // Should consider refactoring apply method later...

    double reli_current;
    double cost_current;

    props.clear();
    values.clear();

    // bad smells everywhere...

    for(std::map<std::string, double>::const_iterator it = tasks.begin();
                it != tasks.end(); it++) {
            props.push_back(it->first);
            values.push_back(it->second);
    }

    reli_current = reliability_expression.apply(props, values);
    
    
    for(std::map<std::string, double>::const_iterator it = tasks.begin();
                it != tasks.end(); it++) {
            if(isCost(it->first)) {
                props.push_back(it->first);
                values.push_back(it->second);
            }
    }
    
    cost_current = cost_expression.apply(props, values);
    
    this->reli_error = this->reli_value - reli_current;
    this->cost_error = this->cost_value - cost_current; 

    plan(id, cost_current, reli_current);
}

/** **************************************************************
 *                          PLAN
/* ***************************************************************
 * exhaustively analyze whether an action fits the setpoints
 * ***************************************************************
*/
void ControllerNode::plan(std::string id, double ccurrent, double rcurrent) {
    double action;

    if (this->reli_error > 0) {
        if(this->reli_error < 0.05) {
            action = 0.01;
        } else if (this->reli_error < 0.1){
            action = 0.05;
        } else {
            action = 0.1;
        }
    } else {
        if(this->reli_error < -0.05) {
            action = -0.01;
        } else if (this->reli_error < -0.1){
            action = -0.05;
        } else {
            action = -0.1;
        }
    }
    execute(id, action, ccurrent, rcurrent);
}

/** **************************************************************
 *                         EXECUTE
/* ***************************************************************
 * if so, send messages containing the actions to the modules
 * ***************************************************************
*/
void ControllerNode::execute(std::string id, double action, double ccurrent, double rcurrent) {

    bsn::operation::Operation op = bsn::operation::Operation();

    int sensor_id = stoi((op.split(id, '.'))[0]);
    std::string current_sensor;

    switch(sensor_id) {
        case 1:
            current_sensor = "oximeter";
            break;
        case 2:
            current_sensor = "ecg";
            break;
        case 3:
            current_sensor = "thermometer";
            break;
        case 4:
            current_sensor = "abp";
            break;
        case 5: 
            current_sensor = "acc";
            break;
    }

    ros::NodeHandle publisher_handler;
	ros::Publisher actuator_pub = publisher_handler.advertise<bsn::ControlCommand>("controller_command", 1000);

    bsn::ControlCommand command_msg;

    command_msg.active = true;
    command_msg.frequency = action;

    actuator_pub.publish(command_msg);

    ros::Publisher centralhub_pub = publisher_handler.advertise<bsn::SystemInfo>("system_info", 1000);

    bsn::SystemInfo centralhub_msg;

    centralhub_msg.cost = ccurrent;
    centralhub_msg.reliability = rcurrent;

    centralhub_pub.publish(centralhub_msg);

}

void ControllerNode::run(){

    ros::NodeHandle n;
   
    ros::Subscriber t_sub = n.subscribe("manager_sensor", 1000, &ControllerNode::receiveTaskInfo, this);
    ros::Subscriber c_sub = n.subscribe("manager_sensor", 1000, &ControllerNode::receiveContextInfo, this);

    ros::spin();

    return;
}
