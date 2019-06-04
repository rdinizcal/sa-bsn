#include "ControllerNode.hpp"
#include "operation/Operation.hpp"
#include <ros/package.h>
#include <memory>

using namespace bsn::goalmodel;

ControllerNode::ControllerNode(int  &argc, char **argv, std::string name):
    tasks(),
    contexts(),
    cost_expression(),
    reliability_expression() 
    {}

ControllerNode::~ControllerNode() {}

void ControllerNode::setTaskValue(std::string id, double value) {
    this->tasks[id] = value;
}

double ControllerNode::getTaskValue(std::string id) {
    return this->tasks[id];
}

bool ControllerNode::isCost(std::string id) {
    return id[0] == ('W');
}

std::string ControllerNode::getSensorName(const int sensor_id) {
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
        case 41:
            current_sensor = "abp";
            break;
        case 42:
            current_sensor = "abp";
            break;
        case 5: 
            current_sensor = "acc";
            break;
    }

    return current_sensor;
}

std::string ControllerNode::getTaskActuator(std::string id) {
    bsn::operation::Operation op = bsn::operation::Operation();

    std::string sensor_name;
    std::string current_sensor;
    std::string actuator_name;
    int sensor_id;
    
    sensor_name = (op.split(id, '.'))[1];
    sensor_id = std::stoi(sensor_name) / 10;

    current_sensor = getSensorName(sensor_id);

    actuator_name = current_sensor + "_control_command";

    return actuator_name;
}

std::string ControllerNode::getContextActuator(std::string id) {
    bsn::operation::Operation op = bsn::operation::Operation();

    std::string sensor_name;
    std::string current_sensor;
    std::string actuator_name;
    int sensor_id;
    
    sensor_name = (op.split(id, '_'))[3];
    sensor_id = std::stoi(sensor_name);

    current_sensor = getSensorName(sensor_id);

    actuator_name = current_sensor + "_control_command";

    return actuator_name;
}

void ControllerNode::setUp() {

    GoalTree goalModel("Body Sensor Network");
    std::string path = ros::package::getPath("manager");

    // Set up the goal tree goalModel        
    LeafTask g3_t1_11("G3_T1.11","Read data", Property("W_G3_T1_11", 1),Property("R_G3_T1_11", 1),Property("F_G3_T1_11",1));
    LeafTask g3_t1_12("G3_T1.12","Filter data", Property("W_G3_T1_12", 1),Property("R_G3_T1_12", 1),Property("F_G3_T1_12",1));
    LeafTask g3_t1_13("G3_T1.13","Transfer data", Property("W_G3_T1_13", 1),Property("R_G3_T1_13", 1),Property("F_G3_T1_13",1));

    g3_t1_11.setContext(bsn::goalmodel::Context("CTX_G3_T1_1","SaO2_available", true));
    g3_t1_12.setContext(bsn::goalmodel::Context("CTX_G3_T1_1","SaO2_available", true));
    g3_t1_13.setContext(bsn::goalmodel::Context("CTX_G3_T1_1","SaO2_available", true));

    Task g3_t1_1("G3_T1.1", "Collect SaO2 data");
    g3_t1_1.addChild(std::make_shared<LeafTask>(g3_t1_11));
    g3_t1_1.addChild(std::make_shared<LeafTask>(g3_t1_12));
    g3_t1_1.addChild(std::make_shared<LeafTask>(g3_t1_13));

    LeafTask g3_t1_21("G3_T1.21","Read data", Property("W_G3_T1_21", 1),Property("R_G3_T1_21", 1),Property("F_G3_T1_21",1));
    LeafTask g3_t1_22("G3_T1.22","Filter data", Property("W_G3_T1_22", 1),Property("R_G3_T1_22", 1),Property("F_G3_T1_22",1));
    LeafTask g3_t1_23("G3_T1.23","Transfer data", Property("W_G3_T1_23", 1),Property("R_G3_T1_23", 1),Property("F_G3_T1_23",1));

    g3_t1_21.setContext(bsn::goalmodel::Context("CTX_G3_T1_2","ECG_available", true));
    g3_t1_22.setContext(bsn::goalmodel::Context("CTX_G3_T1_2","ECG_available", true));
    g3_t1_23.setContext(bsn::goalmodel::Context("CTX_G3_T1_2","ECG_available", true));

    Task g3_t1_2("G3_T1.2", "Collect ECG data");
    g3_t1_2.addChild(std::make_shared<LeafTask>(g3_t1_21));
    g3_t1_2.addChild(std::make_shared<LeafTask>(g3_t1_22));
    g3_t1_2.addChild(std::make_shared<LeafTask>(g3_t1_23));

    LeafTask g3_t1_31("G3_T1.31","Read data", Property("W_G3_T1_31", 1),Property("R_G3_T1_31", 1),Property("F_G3_T1_31",1));
    LeafTask g3_t1_32("G3_T1.32","Filter data", Property("W_G3_T1_32", 1),Property("R_G3_T1_32", 1),Property("F_G3_T1_32",1));
    LeafTask g3_t1_33("G3_T1.33","Transfer data", Property("W_G3_T1_33", 1),Property("R_G3_T1_33", 1),Property("F_G3_T1_33",1));

    g3_t1_31.setContext(bsn::goalmodel::Context("CTX_G3_T1_3","TEMP_available", true));
    g3_t1_32.setContext(bsn::goalmodel::Context("CTX_G3_T1_3","TEMP_available", true));
    g3_t1_33.setContext(bsn::goalmodel::Context("CTX_G3_T1_3","TEMP_available", true));

    Task g3_t1_3("G3_T1.3", "Collect TEMP data");
    g3_t1_3.addChild(std::make_shared<LeafTask>(g3_t1_31));
    g3_t1_3.addChild(std::make_shared<LeafTask>(g3_t1_32));
    g3_t1_3.addChild(std::make_shared<LeafTask>(g3_t1_33));

    LeafTask g3_t1_411("G3_T1.411","Read diastolic", Property("W_G3_T1_411", 1),Property("R_G3_T1_411", 1),Property("F_G3_T1_411",1));
    LeafTask g3_t1_412("G3_T1.412","Read systolic", Property("W_G3_T1_412", 1),Property("R_G3_T1_412", 1),Property("F_G3_T1_412",1));
    LeafTask g3_t1_42("G3_T1.42","Filter data", Property("W_G3_T1_42", 1),Property("R_G3_T1_42", 1),Property("F_G3_T1_42",1));
    LeafTask g3_t1_43("G3_T1.43","Transfer data", Property("W_G3_T1_43", 1),Property("R_G3_T1_43", 1),Property("F_G3_T1_43",1));
    
    g3_t1_411.setContext(bsn::goalmodel::Context("CTX_G3_T1_4","ABP_available", true));
    g3_t1_412.setContext(bsn::goalmodel::Context("CTX_G3_T1_4","ABP_available", true));
    g3_t1_42.setContext(bsn::goalmodel::Context("CTX_G3_T1_4","ABP_available", true));
    g3_t1_43.setContext(bsn::goalmodel::Context("CTX_G3_T1_4","ABP_available", true));

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
    LeafTask g4_t1_2("G4_T1.2","Filter data", Property("W_G4_T1_2", 1),Property("R_G4_T1_2", 1),Property("F_G4_T1_2",1));
    LeafTask g4_t1_3("G4_T1.3","Transfer data", Property("W_G4_T1_3", 1),Property("R_G4_T1_3", 1),Property("F_G4_T1_3",1));

    Task g4_t1("G4_T1", "Analyze vital signs");
    g4_t1.addChild(std::make_shared<LeafTask>(g4_t1_1));
    g4_t1.addChild(std::make_shared<LeafTask>(g4_t1_2));
    g4_t1.addChild(std::make_shared<LeafTask>(g4_t1_3));

    Goal g4("G4", "Vital signs are analyzed");
    g4.addChild(std::make_shared<Task>(g4_t1));

    Goal g2("G2", "Patient status is monitored");
    g2.addChild(std::make_shared<Goal>(g4));
    g2.addChild(std::make_shared<Goal>(g3));

    Goal g1("G1", "Emergency is detected");
    g1.addChild(std::make_shared<Goal>(g2));

    goalModel.addRootGoal(std::make_shared<Goal>(g1));

    // Set up cost and reliability expressions
    std::ifstream cost_file;
    std::string cost_formula;

    std::ifstream reliability_file;
    std::string reliability_formula;

    try{
        cost_file.open(path + "/formulae/cost.formula");
        std::getline(cost_file,cost_formula);
        cost_file.close();
    } catch (std::ifstream::failure e) { std::cerr << "Exception opening/reading/closing file (cost.formula)\n"; }

    try{
        reliability_file.open(path + "/formulae/reliability.formula");
        std::getline(reliability_file,reliability_formula);
        reliability_file.close();
    } catch (std::ifstream::failure e) { std::cerr << "Exception opening/reading/closing file (reliability.formula)\n"; }

    this->cost_expression = bsn::model::Formula(cost_formula);
    this->reliability_expression = bsn::model::Formula(reliability_formula);

    std::vector<std::shared_ptr<bsn::goalmodel::LeafTask>> leafTasks; 

    leafTasks = goalModel.getLeafTasks();

    //set initial conditions: order should be respected

    // Get context information from leafTasks

    bsn::goalmodel::Context aux_context;
    std::string newName;
    std::string contextName;

    for(std::shared_ptr<bsn::goalmodel::LeafTask> it : leafTasks) 
    {
        
        newName = it->getContext().getID();

        if(newName != ""){
            std::replace(newName.begin(),newName.end(),  '.', '_');
            aux_context.setDescription(it->getContext().getDescription());
            aux_context.setValue(it->getContext().getValue());
            aux_context.setID(newName);
            contexts[newName] = aux_context;
        }
    }        

    //reli expr
    for(std::shared_ptr<bsn::goalmodel::LeafTask> it : leafTasks) 
    {
        setTaskValue(it->getReliability().getID(), it->getReliability().getValue());
        setTaskValue(it->getFrequency().getID(), it->getFrequency().getValue());
    }

    std::map<std::string, double>::const_iterator it1;
    std::map<std::string, bsn::goalmodel::Context>::const_iterator it2;

    for(it1 = tasks.begin(); it1 != tasks.end(); it1++)
    {
        props.push_back(it1->first);
        values.push_back(it1->second);
    }

    for(it2 = contexts.begin(); it2 != contexts.end(); it2++)
    {
        props.push_back(it2->first);
        values.push_back((double)(it2->second.getValue()));
    }

    desired_reli = reliability_expression.apply(props, values);

    props.clear(); values.clear();
    
    //cost expr
    for(std::shared_ptr<bsn::goalmodel::LeafTask> it : leafTasks) 
    {
        setTaskValue(it->getReliability().getID(), it->getReliability().getValue());
        setTaskValue(it->getFrequency().getID(), it->getFrequency().getValue());
        setTaskValue(it->getCost().getID(), it->getCost().getValue());
    }

    for(it1 = tasks.begin(); it1 != tasks.end(); it1++)
    {
        props.push_back(it1->first);
        values.push_back(it1->second);
    }

    for(it2 = contexts.begin(); it2 != contexts.end(); it2++)
    {
        props.push_back(it2->first);
        values.push_back((double)(it2->second.getValue()));
    }        

    desired_cost = cost_expression.apply(props, values);

    ros::NodeHandle publisher_handler;

    centralhub_pub = publisher_handler.advertise<bsn::SystemInfo>("system_info", 1000);
    ecg_pub = publisher_handler.advertise<bsn::ControlCommand>("ecg_control_command", 1000);
    oxi_pub = publisher_handler.advertise<bsn::ControlCommand>("oximeter_control_command", 1000);
    abp_pub = publisher_handler.advertise<bsn::ControlCommand>("abp_control_command", 1000);
    therm_pub = publisher_handler.advertise<bsn::ControlCommand>("thermometer_control_command", 1000);

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

    message_id = msg->task_id;
    std::string id_aux = message_id;
    std::replace(id_aux.begin(), id_aux.end(), '.', '_');
    std::string costID = "W_" + id_aux;
    std::string reliID = "R_" + id_aux;
    std::string freqID = "F_" + id_aux;

/*      
    std::cout << "costID = " << msg->cost << std::endl;
    std::cout << "reliID = " << msg->reliability << std::endl;
    std::cout << "freqID = " << msg->frequency << std::endl;
*/ 

    setTaskValue(costID, msg->cost);
    setTaskValue(reliID, msg->reliability);
    setTaskValue(freqID, msg->frequency);

    analyze();
}

void ControllerNode::receiveContextInfo(const bsn::ContextInfo::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->context_id.c_str());

    message_id = msg->context_id;
    std::string id_aux = message_id;
    std::replace(id_aux.begin(), id_aux.end(), '.', '_');

    bool status = msg->status;

    contexts[id_aux].setValue(msg->status);

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
    
    // Should consider refactoring apply method later...

    std::map<std::string, double>::const_iterator it1;
    std::map<std::string, bsn::goalmodel::Context>::const_iterator it2;

    // clear previous values and props
    props.clear(); values.clear();

    // bad smells everywhere...

    for(it1 = tasks.begin(); it1 != tasks.end(); it1++)
    {
        if(!isCost(it1->first)) {
            props.push_back(it1->first);
            values.push_back(it1->second);
        }
    }

    for(it2 = contexts.begin(); it2 != contexts.end(); it2++)
    {
        props.push_back(it2->first);
        values.push_back((double)(it2->second.getValue()));
    }      

    current_reli = reliability_expression.apply(props, values);

    props.clear(); values.clear();


    for(it1 = tasks.begin(); it1 != tasks.end(); it1++)
    {
        props.push_back(it1->first);
        values.push_back(it1->second);
    }

    for(it2 = contexts.begin(); it2 != contexts.end(); it2++)
    {
        props.push_back(it2->first);
        values.push_back((double)(it2->second.getValue()));
    }      

    current_cost = cost_expression.apply(props, values);

    reli_error = desired_reli - current_reli;
    cost_error = desired_cost - current_cost;

    plan();
}

/** **************************************************************
 *                          PLAN
/* ***************************************************************
 * exhaustively analyze whether an action fits the setpoints
 * ***************************************************************
*/
void ControllerNode::plan() {
    double action;

    if (reli_error > 0) {
        if(reli_error < 0.05) {
            action = 0.01;
        } else if (reli_error < 0.1){
            action = 0.05;
        } else {
            action = 0.1;
        }
    } else if (reli_error < 0) {
        if(reli_error < -0.05) {
            action = -0.01;
        } else if (reli_error < -0.1){
            action = -0.05;
        } else {
            action = -0.1;
        }
    } else action = 0.0;

    execute(action);
}

/** **************************************************************
 *                         EXECUTE
/* ***************************************************************
 * if so, send messages containing the actions to the modules
 * ***************************************************************
*/
void ControllerNode::execute(double action) {

    bsn::operation::Operation op = bsn::operation::Operation();
    std::string actuator_name;
    std::string message_type;

    ros::NodeHandle publisher_handler;

    bsn::ControlCommand command_msg;
    bsn::SystemInfo centralhub_msg;

    message_type = op.split(message_id, '_')[0];
    actuator_name = message_type == "CTX" ? getContextActuator(message_id) : getTaskActuator(message_id);

    if (action != 0) 
    {
        std::cout << "sending action: " << action << " to "  << actuator_name << std::endl;

        command_msg.active = true;
        command_msg.frequency = action;

        if(actuator_name == "ecg_control_command") {
            ecg_pub.publish(command_msg);
        } else  if(actuator_name == "abp_control_command"){
            abp_pub.publish(command_msg);
        } else  if(actuator_name == "oximeter_control_command"){
            oxi_pub.publish(command_msg);
        } else  if(actuator_name == "thermometer_control_command"){
            therm_pub.publish(command_msg);
        }
    }

    centralhub_msg.cost = current_cost;
    centralhub_msg.reliability = current_reli;

    centralhub_pub.publish(centralhub_msg);

    return;
}

void ControllerNode::run(){

    ros::NodeHandle n;
   
    ros::Subscriber t_sub = n.subscribe("task_info", 1000, &ControllerNode::receiveTaskInfo, this);
    ros::Subscriber c_sub = n.subscribe("context_info", 1000, &ControllerNode::receiveContextInfo, this);

    ros::spin();

    return;
}
