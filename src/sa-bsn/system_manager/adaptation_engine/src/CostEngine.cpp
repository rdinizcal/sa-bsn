#include "engine/CostEngine.hpp"

using namespace bsn::goalmodel;

struct comp{
    template<typename T>
    bool operator()(const T& l, const T& r) const
    {
        if (l.second != r.second)
            return l.second < r.second;
 
        return l.first < r.first;
    }
};

CostEngine::CostEngine(int  &argc, char **argv, std::string name): ROSComponent(argc, argv, name), r_ref(0.9), stability_margin(0.02), offset(0), info_quant(0), monitor_freq(1), actuation_freq(1), target_system_model(), strategy(),  priority(),  Kp(0.01), cycles(0) {}

CostEngine::~CostEngine() {}

std::string fetch_formula(std::string name){
    ros::NodeHandle client_handler;
    ros::ServiceClient client_module;

    client_module = client_handler.serviceClient<archlib::DataAccessRequest>("DataAccessRequest");

    archlib::DataAccessRequest r_srv;
    r_srv.request.name = ros::this_node::getName();
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
void CostEngine::setUp_formula(std::string formula_str) {
    target_system_model = bsn::model::Formula(formula_str);
    
    // Extracts the terms that will compose the strategy
    std::vector<std::string> terms = target_system_model.getTerms();

    strategy = initialize_strategy(terms, 0);

    // Initializes the target system model
    calculate_qos(target_system_model,strategy);

    priority = initialize_priority(terms, 50, "W_");

    return;
}

void CostEngine::setUp() {
    ros::NodeHandle nh;
    nh.getParam("setpoint", c_ref);
	nh.getParam("offset", offset);
	nh.getParam("gain", Kp);
	nh.getParam("info_quant", info_quant);
	nh.getParam("monitor_freq", monitor_freq);
    rosComponentDescriptor.setFreq(monitor_freq);
	nh.getParam("actuation_freq", actuation_freq);

    std::string formula_str = "";
    do{
        formula_str = fetch_formula("cost");
        ros::Duration(1.0).sleep() ;
    } while(formula_str=="");
    
    setUp_formula(formula_str);

    enact = handle.advertise<archlib::Strategy>("strategy", 10);
    energy_status = handle.advertise<archlib::EnergyStatus>("log_energy_status", 10);

    enactor_server = handle.advertiseService("EngineRequest", &Engine::sendAdaptationParameter, this);
}

void CostEngine::tearDown() {}

bool CostEngine::sendAdaptationParameter(archlib::EngineRequest::Request &req, archlib::EngineRequest::Response &res) {
    try {
        res.content = "cost";
    } catch(...) {}
}

void CostEngine::receiveException(const archlib::Exception::ConstPtr& msg){
    std::string content = msg->content.c_str();

    std::vector<std::string> param = bsn::utils::split(content, '=');

    // /g3t1_1
    std::string first = param[0];
    std::transform(first.begin(), first.end(),first.begin(), ::toupper); // /G3T1_1
    first.erase(0,1); // G3T1_1
    first.insert(int(first.find('T')), "_"); // G3_T1_1
    first = "W_" + first;

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
double CostEngine::calculate_qos(bsn::model::Formula model, std::map<std::string, double> conf) {
    model.setTermValueMap(conf);
    return model.evaluate();
}

/*bool CostEngine::blacklisted(std::map<std::string,double> &strat) {
    for (std::vector<std::map<std::string,double>>::iterator it = blacklist.begin(); it != blacklist.end(); it++){
        if(*it == strat) return true;
    }

    return false;
}*/

/** **************************************************************
 *                          MONITOR_COST
/* ***************************************************************
 * ***************************************************************
*/ 
void CostEngine::monitor() {
    std::cout << "[monitoring]" << std::endl;
    cycles++;

    // request system status data for the knowledge repository
    ros::NodeHandle client_handler;
    ros::ServiceClient client_module;

    client_module = client_handler.serviceClient<archlib::DataAccessRequest>("DataAccessRequest");
    
    //reset the formula_str
    for (std::map<std::string,double>::iterator it = strategy.begin(); it != strategy.end(); ++it){
        if(it->first.find("CTX_") != std::string::npos)  it->second = 0;
        if(it->first.find("W_") != std::string::npos)  it->second = 1;
        //if(it->first.find("F_") != std::string::npos)  it->second = 1;
    }

    archlib::DataAccessRequest r_srv;
    r_srv.request.name = ros::this_node::getName();
    r_srv.request.query = "all:cost:" + std::to_string(info_quant);

    if(!client_module.call(r_srv)) {
        ROS_ERROR("Failed to connect to data access node.");
        return;
    } /*request cost for all tasks*/
    
    //expecting smth like: "/g3t1_1:success,fail,success;/g4t1:success; ..."
    std::string ans = r_srv.response.content;

    if(ans == ""){
        ROS_ERROR("Received empty answer when asked for cost.");
    }

    std::vector<std::string> pairs = bsn::utils::split(ans, ';');
    
    for (std::string it : pairs) {
        std::vector<std::string> pair = bsn::utils::split(it, ':');
        std::string first = pair[0];
        std::string second = pair[1];

        //a "/g3t1_1 arrives here"
        std::transform(first.begin(), first.end(),first.begin(), ::toupper); // /G3T1_1
        first.erase(0,1); // G3T1_1
        first.insert(int(first.find('T')),"_"); // G3_T1_1

        std::vector<std::string> values = bsn::utils::split(second, ',');

        strategy["W_" + first] = stod(values[values.size()-1]);
        std::cout << "W_" + first + " = " << strategy["W_" + first] << std::endl;
    } 

    //request context status for all tasks
    archlib::DataAccessRequest c_srv;
    c_srv.request.name = ros::this_node::getName();
    c_srv.request.query = "all:event:1";

    if (!client_module.call(c_srv)) {
        ROS_ERROR("Failed to connect to data access node.");
        return;
    } 
    
    //expecting smth like: "/g3t1_1:activate;/g4t1:deactivate; ..."
    ans = c_srv.response.content;
    //std::cout << "received=> [" << ans << "]" << std::endl;

    if (ans == "") {
        ROS_ERROR("Received empty answer when asked for event.");
    }

    std::vector<std::string> ctx_pairs = bsn::utils::split(ans, ';');
    
    for (std::string ctx : ctx_pairs) {
        std::vector<std::string> pair = bsn::utils::split(ctx, ':');
        std::string first = pair[0];
        std::string second = pair[1];

        //a "/g3t1_1 arrives here"
        std::transform(first.begin(), first.end(),first.begin(), ::toupper); // /G3T1_1
        first.erase(0,1); // G3T1_1
        first.insert(int(first.find('T')),"_"); // G3_T1_1

        std::vector<std::string> values = bsn::utils::split(second, ',');
        for (std::string value : values) {
            if (first != "G4_T1") {
                strategy["CTX_" + first] = 1;
                
                if (value == "deactivate") {
                    strategy["W_" + first] = 0;
                    deactivatedComponents["W_" + first] = 1;
                    std::cout << first + " was deactivated and its cost was set to 0" << std::endl;
                }
            } else {
                if (value == "activate") {
                    strategy["CTX_" + first] = 1;
                    deactivatedComponents["W_" + first] = 0;
                } else {
                    strategy["CTX_" + first] = 0;
                    deactivatedComponents["W_" + first] = 1;
                }
            }
        }
    }
    
    analyze();
}

/** **************************************************************
 *                         ANALYZE
/* ***************************************************************
 * ***************************************************************
*/ 
void CostEngine::analyze() {
    std::cout << "[analyze]" << std::endl;

    //std::cout << "strategy: [";
    //for (std::map<std::string,double>::iterator itt = strategy.begin(); itt != strategy.end(); ++itt) {
    //   std::cout<< itt->first << ":" << itt->second << ", ";
    //}

    double r_curr, c_curr;
    double error;
    c_curr = calculate_qos(target_system_model,strategy);
    std::cout << "current system cost: " <<  c_curr << std::endl;
    
    archlib::EnergyStatus msg;
    msg.source = rosComponentDescriptor.getName();
    msg.content = "global:" + std::to_string(c_curr) + ";";

    for (std::map<std::string,double>::iterator it = strategy.begin(); it != strategy.end(); ++it) {
        if(it->first.find("W_") != std::string::npos) {
            std::string component_name = it->first.substr(2,it->first.size()-2);
            for(auto& c : component_name) {
                c = std::tolower(c);
            }
            component_name.erase(component_name.begin()+2);
            component_name = "/" + component_name;

            msg.content += component_name + ":" + std::to_string(it->second) + ";";
        }
    }

    energy_status.publish(msg);

    error = c_ref - c_curr;
    // if the error is out of the stability margin, plan!
    if ((error > c_ref * stability_margin) || (error < -stability_margin * c_ref)) {
        if (cycles >= monitor_freq / actuation_freq) {
            cycles = 0;
            plan();
        }
    }
}

/** **************************************************************
 *                          PLAN_COST
/* ***************************************************************
 * ***************************************************************
*/
void CostEngine::plan() {
    std::cout << "[plan]" << std::endl;

    std::cout << "c_ref= " << c_ref << std::endl;
    double c_curr = calculate_qos(target_system_model,strategy);
    std::cout << "c_curr= " << c_curr << std::endl;
    double error = c_ref - c_curr;
    std::cout << "error= " << error << std::endl;

    //reset the formula_str
    std::vector<std::string> c_vec;
    for (std::map<std::string,double>::iterator it = strategy.begin(); it != strategy.end(); ++it) {
        if(it->first.find("W_") != std::string::npos) {
            std::string task = it->first;
            task.erase(0,2);
            if((strategy["CTX_"+task] != 0) /*&& (strategy["F_"+task] != 0)*/){ // avoid ctx = 0 components thus infinite loops
                if(!deactivatedComponents[it->first]) {
                    c_vec.push_back(it->first);
                    strategy[it->first] = c_curr;
                } else {
                    strategy[it->first] = 0;
                }
            }
        }
    }  

    int sensor_num;
    if(c_vec.size()-1 < 1) {
        sensor_num = 1;
    } else {
        sensor_num = c_vec.size()-1;
    }

    for(std::map<std::string,double>::iterator it = strategy.begin(); it != strategy.end(); ++it) {
        if(it->first.find("W_") != std::string::npos) {
            it->second = it->second/sensor_num;
        }
    }

    //Divide error and ref by the number of sensors
    error /= sensor_num; 
    c_ref /= sensor_num;

    //reorder r_vec based on priority
    std::vector<std::string> aux = c_vec;
    c_vec.clear();
    std::set<std::pair<std::string,int>, comp> set(priority.begin(), priority.end());
    for(auto const &pair: set){
        if(!deactivatedComponents["W_" + pair.first] && std::find(aux.begin(), aux.end(), pair.first) != aux.end()){ // key from priority exists in r_vec
            c_vec.push_back(pair.first);
        }
    }

    //print ordered r_vec
    std::cout << "ordered c_vec: [";
    for(std::vector<std::string>::iterator i = c_vec.begin(); i != c_vec.end(); ++i) { 
        std::cout << *i << ", ";
    }
    std::cout << "]" << std::endl;

    // ladies and gentleman, the search...

    std::vector<std::map<std::string, double>> solutions;
    for(std::vector<std::string>::iterator i = c_vec.begin(); i != c_vec.end(); ++i) { 
        //std::cout <<"i: "<< *i << std::endl;

        //reset offset
        for (std::vector<std::string>::iterator it = c_vec.begin(); it != c_vec.end(); ++it) {
            if(*it != "W_G4_T1") {
                if(error>0){
                    strategy[*it] = strategy[*it]*(1-offset);
                } else if(error<0) {
                    strategy[*it] = strategy[*it]*(1+offset);
                }
            } else {
                strategy[*it] = 0;
            }
        }
        double c_new = calculate_qos(target_system_model,strategy); // set offset
        c_new /= sensor_num;
        std::cout << "offset=" << c_new << std::endl;

        std::map<std::string,double> prev;
        double c_prev=0;
        if(error > 0){
            do {
                prev = strategy;
                c_prev = c_new;
                if(*i != "W_G4_T1") {
                    strategy[*i] += Kp*error;
                } else {
                    strategy[*i] = 0;
                }
                c_new = calculate_qos(target_system_model,strategy);
                c_new /= sensor_num;
            } while(c_new < c_ref && c_prev < c_new && strategy[*i] > 0);
        } else if (error < 0){
            do {
                prev = strategy;
                c_prev = c_new;
                if(*i != "W_G4_T1") {
                    strategy[*i] += Kp*error;
                } else {
                    strategy[*i] = 0;
                }
                c_new = calculate_qos(target_system_model,strategy);
                c_new /= sensor_num;
            } while(c_new > c_ref && c_prev > c_new && strategy[*i] > 0);
        }

        strategy = prev;
        c_new = calculate_qos(target_system_model,strategy);
        c_new /= sensor_num;

        for(std::vector<std::string>::iterator j = c_vec.begin(); j != c_vec.end(); ++j) { // all the others
            if(*i == *j) continue;
            //std::cout <<"j: "<< *j << std::endl;

            if(error > 0){
                do {
                    prev = strategy;
                    c_prev = c_new;
                    if(*j != "W_G4_T1") {
                        strategy[*j] += Kp*error;
                    } else {
                        strategy[*i] = 0;
                    }
                    c_new = calculate_qos(target_system_model,strategy);
                    c_new /= sensor_num;
                } while(c_new < c_ref && c_prev < c_new && strategy[*j] > 0);
            } else if (error < 0) {
                do {
                    prev = strategy;
                    c_prev = c_new;
                    if(*i != "W_G4_T1") {
                        strategy[*i] += Kp*error;
                    } else {
                        strategy[*i] = 0;
                    }
                    c_new = calculate_qos(target_system_model,strategy);
                    c_new /= sensor_num;
                } while(c_new > c_ref && c_prev > c_new && strategy[*j] > 0);
            }
            
            strategy = prev;
            c_new = calculate_qos(target_system_model,strategy);
            c_new /= sensor_num;
        }

        bool negative_cost = false;
        for(std::map<std::string,double>::iterator strategy_it = strategy.begin();strategy_it != strategy.end();++strategy_it) {
            if(strategy_it->first.find("W_") != std::string::npos) {
                if(strategy_it->second < 0) {
                    negative_cost = true;
                    break;
                }
            }
        }

        if(!negative_cost) {
            solutions.push_back(strategy);
        }
    }

    c_ref *= sensor_num;

    for (std::vector<std::map<std::string,double>>::iterator it = solutions.begin(); it != solutions.end(); it++){
        strategy = *it;
        double c_new = calculate_qos(target_system_model,strategy);

        std::cout << "strategy: [";
        for (std::map<std::string,double>::iterator itt = (*it).begin(); itt != (*it).end(); ++itt) {
            if(itt->first.find("W_") != std::string::npos) {
                std::cout<< itt->first << ":" << itt->second << ", ";
            }
        }
        std::cout << "] = " << c_new << std::endl;

        if(/*!blacklisted(*it) && */(c_new > c_ref*(1-stability_margin) && c_new < c_ref*(1+stability_margin))){ // if not listed and converges, yay!
            execute();
            return;
        }
    }

    ROS_INFO("Did not converge :(");
}

/** **************************************************************
 *                         EXECUTE
/* ***************************************************************
 * if so, send messages containing the actions to the modules
 * ***************************************************************
*/
void CostEngine::execute() {
    std::cout << "[execute]" << std::endl;

    std::string content = "";
    size_t index = 0;
    bool flag = false;  //nasty
    bool flagO = false; //uber nasty
    
    //get the costs in the formula_str and send!
    // send in form "/g3t1_1:0.89;/g4t1=0.2;..."
    for (std::map<std::string,double>::iterator it = strategy.begin(); it != strategy.end(); ++it) {
        std::string aux = "";
        if(it->first.find("W_") != std::string::npos) { //if it is a W_ term of the formula_str 
            aux += it->first;
            std::transform(aux.begin(), aux.end(), aux.begin(), ::tolower);

            std::vector<std::string> str = bsn::utils::split(aux, '_');
            content += "/";
            content += str[1]; //g3
            content += str[2] ; //t1
            if(str.size()>3) content += "_" + str[3]; //_1:
            content += ":"; 
            content += std::to_string(it->second) + ","; // 0.82 
            flag = true;
        }
        if(flag){
            //remove last ','
            content.pop_back();
            content += ";";
            flag = false;
            flagO = true;
        }
        aux.clear();
    }

    //remove last ';'
    if(flagO) content.pop_back();

    archlib::Strategy msg;
    msg.source = ros::this_node::getName();
    msg.target = "/enactor";
    msg.content = content;

    enact.publish(msg);

    std::cout << "[ " << content << "]" << std::endl;

    return;
}

void CostEngine::body(){
    ros::NodeHandle n;
    ros::Subscriber t_sub = n.subscribe("exception", 1000, &Engine::receiveException, this);

    ros::Rate loop_rate(rosComponentDescriptor.getFreq());
    int update=0;
    while (ros::ok){
        update++;
        if (update >= rosComponentDescriptor.getFreq()*10){
            update = 0;
            std::string formula_str = fetch_formula("cost");
            if(formula_str=="") continue;
            setUp_formula(formula_str);
        }

        monitor();

        ros::spinOnce();
        loop_rate.sleep();        
    }   

    return;
}
