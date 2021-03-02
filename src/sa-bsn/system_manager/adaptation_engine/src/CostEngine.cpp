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

CostEngine::CostEngine(int  &argc, char **argv, std::string name): Engine(argc, argv, name), setpoint(), offset(), gain(), tolerance(0.02), prefix("W_"), enact(), energy_status() {}

CostEngine::~CostEngine() {}

void CostEngine::setUp() {
    Engine::setUp();
    ros::NodeHandle handle;
    handle.getParam("setpoint", setpoint);
	handle.getParam("offset", offset);
	handle.getParam("gain", gain);

    enact = handle.advertise<archlib::Strategy>("strategy", 10);
    energy_status = handle.advertise<archlib::EnergyStatus>("log_energy_status", 10);
}

void CostEngine::tearDown() {
    Engine::tearDown();
}


std::string CostEngine::get_prefix() {
    return prefix;
}

/**
   Returns an initialized strategy with init_value values.
   @param terms The terms that compose the strategy.
   @return The map containing terms of the formula and initial values.
 */
std::map<std::string, double> CostEngine::initialize_strategy(std::vector<std::string> terms){
    std::map<std::string, double> strategy;
    
    for (std::vector<std::string>::iterator it = terms.begin(); it != terms.end(); ++it) {
        strategy[*it] = 0;
    }

    return strategy;
}

/**
   Returns an initialized priorities vector with init_value values.
   @param terms The terms that compose the strategy.
   @param prefix A prefix of the term (e.g., R_ for reliability and W_ for cost).
   @return The map containing terms of the formula and initial values.
 */
std::map<std::string, int> CostEngine::initialize_priority(std::vector<std::string> terms) {
    std::map<std::string, int> priority;
    
    for (std::vector<std::string>::iterator it = terms.begin(); it != terms.end(); ++it) {
        if((*it).find("W_") != std::string::npos) {
            priority[*it] = 50;
        }
    }

    return priority;
}

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
    r_srv.request.name = "/engine";
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
    c_srv.request.name = "/engine";
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
    msg.source = "/engine";
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

    error = setpoint - c_curr;
    // if the error is out of the stability margin, plan!
    if ((error > setpoint * tolerance) || (error < -tolerance * setpoint)) {
        if (cycles >= monitor_freq / actuation_freq) {
            cycles = 0;
            plan();
        }
    }
}

void CostEngine::plan() {
    std::cout << "[plan]" << std::endl;

    std::cout << "setpoint= " << setpoint << std::endl;
    double c_curr = calculate_qos(target_system_model,strategy);
    std::cout << "c_curr= " << c_curr << std::endl;
    double error = setpoint - c_curr;
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

    //Divide error and setpoint by the number of sensors
    error /= sensor_num; 
    setpoint /= sensor_num;

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
                    strategy[*i] += gain*error;
                } else {
                    strategy[*i] = 0;
                }
                c_new = calculate_qos(target_system_model,strategy);
                c_new /= sensor_num;
            } while(c_new < setpoint && c_prev < c_new && strategy[*i] > 0);
        } else if (error < 0){
            do {
                prev = strategy;
                c_prev = c_new;
                if(*i != "W_G4_T1") {
                    strategy[*i] += gain*error;
                } else {
                    strategy[*i] = 0;
                }
                c_new = calculate_qos(target_system_model,strategy);
                c_new /= sensor_num;
            } while(c_new > setpoint && c_prev > c_new && strategy[*i] > 0);
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
                        strategy[*j] += gain*error;
                    } else {
                        strategy[*i] = 0;
                    }
                    c_new = calculate_qos(target_system_model,strategy);
                    c_new /= sensor_num;
                } while(c_new < setpoint && c_prev < c_new && strategy[*j] > 0);
            } else if (error < 0) {
                do {
                    prev = strategy;
                    c_prev = c_new;
                    if(*i != "W_G4_T1") {
                        strategy[*i] += gain*error;
                    } else {
                        strategy[*i] = 0;
                    }
                    c_new = calculate_qos(target_system_model,strategy);
                    c_new /= sensor_num;
                } while(c_new > setpoint && c_prev > c_new && strategy[*j] > 0);
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

    setpoint *= sensor_num;

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

        if(/*!blacklisted(*it) && */(c_new > setpoint*(1-tolerance) && c_new < setpoint*(1+tolerance))){ // if not listed and converges, yay!
            execute();
            return;
        }
    }

    ROS_INFO("Did not converge :(");
}

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
    msg.source = "/engine";
    msg.target = "/enactor";
    msg.content = content;

    enact.publish(msg);

    std::cout << "[ " << content << "]" << std::endl;

    return;
}