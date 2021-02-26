#include "engine/Engine.hpp"

using namespace bsn::goalmodel;

Engine::Engine(int  &argc, char **argv, std::string name): ROSComponent(argc, argv, name), ref(0.9), stability_margin(0.02), offset(0), info_quant(0), monitor_freq(1), actuation_freq(1), target_system_model(), strategy(),  priority(),  Kp(0.01), cycles(0) {}

Engine::~Engine() {}

std::string fetch_formula(std::string name){
    ros::NodeHandle client_handler;
    ros::ServiceClient client_module;

    client_module = client_handler.serviceClient<archlib::DataAccessRequest>("DataAccessRequest");

    archlib::DataAccessRequest r_srv;
    r_srv.request.name = "/engine";
    r_srv.request.query = name + "_formula";

    if(!client_module.call(r_srv)) {
        ROS_ERROR("Tried to fetch formula string, but Data Access is not responding.");
        return "";
    }
    
    std::string formula_str = r_srv.response.content;

    if(formula_str == ""){
        ROS_ERROR("ERROR: Empty formula string received.");
    }

    return formula_str;
}

/**
   Returns an initialized strategy with init_value values.
   @param terms The terms that compose the strategy.
   @param init_value Is the value that will be used to initialize the strategy.
   @return The map containing terms of the formula and initial values.
 */
std::map<std::string, double> initialize_strategy(std::vector<std::string> terms, double init_value){
    std::map<std::string, double> strategy;
    
    for (std::vector<std::string>::iterator it = terms.begin(); it != terms.end(); ++it) {
        strategy[*it] = init_value;
    }

    return strategy;
}

/**
   Returns an initialized priorities vector with init_value values.
   @param terms The terms that compose the strategy.
   @param init_value Is the value that will be used to initialize the strategy.
   @param prefix A prefix of the term (e.g., R_ for reliability and W_ for cost).
   @return The map containing terms of the formula and initial values.
 */
std::map<std::string, int> initialize_priority(std::vector<std::string> terms, int init_value, std::string prefix) {
    std::map<std::string, int> priority;
    
    for (std::vector<std::string>::iterator it = terms.begin(); it != terms.end(); ++it) {
        if((*it).find(prefix) != std::string::npos) {
            priority[*it] = init_value;
        }
    }

    return priority;
}

/**
   Sets up formula-related structures (i.e, target system model, strategy, and priority)
   @param formula_str A string containing the algebraic formula.
   @return void
 */
void Engine::setUp_formula(std::string formula_str) {
    target_system_model = bsn::model::Formula(formula_str);
    
    // Extracts the terms that will compose the strategy
    std::vector<std::string> terms = target_system_model.getTerms();

    if(qos_attribute == "reliability") { 
        strategy = initialize_strategy(terms, 1);
    } else {
        strategy = initialize_strategy(terms, 0);
    } 

    // Initializes the target system model
    calculate_qos(target_system_model,strategy);

    if(qos_attribute == "reliability") { 
        priority = initialize_priority(terms, 50, "R_");
    } else {
        priority = initialize_priority(terms, 50, "W_");
    }

    return;
}

void Engine::setUp() {
    ros::NodeHandle handle;
    handle.getParam("qos_attribute", qos_attribute);
    handle.getParam("setpoint", ref);
	handle.getParam("offset", offset);
	handle.getParam("gain", Kp);
	handle.getParam("info_quant", info_quant);
	handle.getParam("monitor_freq", monitor_freq);
	handle.getParam("actuation_freq", actuation_freq);

    rosComponentDescriptor.setFreq(monitor_freq);

    std::string formula_str = "";
    do{
        formula_str = fetch_formula(qos_attribute);
        ros::Duration(1.0).sleep() ;
    } while(formula_str=="");
    
    setUp_formula(formula_str);

    enact = handle.advertise<archlib::Strategy>("strategy", 10);
    energy_status = handle.advertise<archlib::EnergyStatus>("log_energy_status", 10);

    enactor_server = handle.advertiseService("EngineRequest", &Engine::sendAdaptationParameter, this);
}

void Engine::tearDown() {}

bool Engine::sendAdaptationParameter(archlib::EngineRequest::Request &req, archlib::EngineRequest::Response &res) {
    try {
        res.content = qos_attribute;
    } catch(...) {}
}

void Engine::receiveException(const archlib::Exception::ConstPtr& msg){
    std::string content = msg->content.c_str();

    std::vector<std::string> param = bsn::utils::split(content, '=');

    // /g3t1_1
    std::string first = param[0];
    std::transform(first.begin(), first.end(),first.begin(), ::toupper); // /G3T1_1
    first.erase(0,1); // G3T1_1
    first.insert(int(first.find('T')), "_"); // G3_T1_1
    if(qos_attribute == "reliability") {
        first = "R_" + first; 
    } else {
        first = "W_" + first;
    }

    if (priority.find(first) != priority.end()) {
        priority[first] += stoi(param[1]);
        if (priority[first] > 99) priority[first] = 100;
        if (priority[first] < 1) priority[first] = 0;
    } else {
        ROS_ERROR("COULD NOT FIND COMPONENT IN LIST OF PRIORITIES.");
    }
}


/**
 * Calculates the overall QoS attribute based on the target system model and configuration of parameters
 * @param model An algebraic target system model that represents the QoS attribute
 * @param conf A map of the target system model terms and values
 * @return The value of QoS attribute given the strategy
 */
double Engine::calculate_qos(bsn::model::Formula model, std::map<std::string, double> conf) {
    model.setTermValueMap(conf);
    return model.evaluate();
}

/*bool Engine::blacklisted(std::map<std::string,double> &strat) {
    for (std::vector<std::map<std::string,double>>::iterator it = blacklist.begin(); it != blacklist.end(); it++){
        if(*it == strat) return true;
    }

    return false;
}*/

void Engine::body(){
    ros::NodeHandle n;
    ros::Subscriber t_sub = n.subscribe("exception", 1000, &Engine::receiveException, this);

    ros::Rate loop_rate(rosComponentDescriptor.getFreq());
    int update=0;
    while (ros::ok){
        update++;
        if (update >= rosComponentDescriptor.getFreq()*10){
            update = 0;
            std::string formula_str = fetch_formula(qos_attribute);
            if(formula_str=="") continue;
            setUp_formula(formula_str);
        }

        monitor();

        ros::spinOnce();
        loop_rate.sleep();        
    }   

    return;
}
