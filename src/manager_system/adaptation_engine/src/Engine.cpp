#include "engine/Engine.hpp"

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

Engine::Engine(int  &argc, char **argv, std::string name): ROSComponent(argc, argv, name), r_ref(0.9), stability_margin(0.02), offset(0), info_quant(0), monitor_freq(1), actuation_freq(1), reliability_expression(), strategy(),  priority(),  Kp(0.01), cycles(0) {}

Engine::~Engine() {}

void Engine::setUp() {
    ros::NodeHandle nh;

	nh.getParam("setpoint", r_ref);
	nh.getParam("offset", offset);
	nh.getParam("gain", Kp);
	nh.getParam("info_quant", info_quant);
	nh.getParam("monitor_freq", monitor_freq);
    rosComponentDescriptor.setFreq(monitor_freq);
	nh.getParam("actuation_freq", actuation_freq);

    enact = handle.advertise<archlib::Strategy>("strategy", 10);
    persist_pub = handle.advertise<archlib::Persist>("persist", 10);

    std::string path = ros::package::getPath("adaptation_engine");

    std::ifstream reliability_file;
    std::string reliability_formula;

    try{
        reliability_file.open(path + "/formulae/reliability.formula");
        std::getline(reliability_file,reliability_formula);
        reliability_file.close();
    } catch (std::ifstream::failure e) { std::cerr << "Exception opening/reading/closing file (reliability.formula)\n"; }

    //this->cost_expression = bsn::model::Formula(cost_formula);
    reliability_expression = bsn::model::Formula(reliability_formula);


    //parse formula:
    std::replace(reliability_formula.begin(), reliability_formula.end(), '+',' ');
    std::replace(reliability_formula.begin(), reliability_formula.end(), '-',' ');
    std::replace(reliability_formula.begin(), reliability_formula.end(), '*',' ');
    std::replace(reliability_formula.begin(), reliability_formula.end(), '/',' ');
    std::replace(reliability_formula.begin(), reliability_formula.end(), '(',' ');
    std::replace(reliability_formula.begin(), reliability_formula.end(), ')',' ');
    std::vector<std::string> terms = bsn::utils::split(reliability_formula, ' ');

    for (std::vector<std::string>::iterator it = terms.begin(); it != terms.end(); ++it) {
        strategy[*it] = 1;
    }

    //initialize:
    calculate_reli();

    //initialize priorities
    for (std::map<std::string,double>::iterator it = strategy.begin(); it != strategy.end(); ++it) {
        if(it->first.find("R_") != std::string::npos) {
            priority[it->first] = 50;
        }
    }  
    
}

void Engine::tearDown() {}

void Engine::receiveException(const archlib::Exception::ConstPtr& msg){
    std::string content = msg->content.c_str();

    std::vector<std::string> param = bsn::utils::split(content, '=');

    // /g3t1_1
    std::string first = param[0];
    std::transform(first.begin(), first.end(),first.begin(), ::toupper); // /G3T1_1
    first.erase(0,1); // G3T1_1
    first.insert(int(first.find('T')),"_"); // G3_T1_1
    first = "R_" + first; 

    if(priority.find(first) != priority.end()){  
        priority[first] += stoi(param[1]);
        if(priority[first]>99) priority[first] = 100;
        if(priority[first]<1) priority[first] = 0;
    } else {
        ROS_ERROR("COULD NOT FIND COMPONENT IN LIST OF PRIORITIES.");
    }
}

double Engine::calculate_reli() {
    std::vector<std::string> keys;
    std::vector<double> values;
    for (std::map<std::string,double>::iterator it = strategy.begin(); it != strategy.end(); ++it) {
        keys.push_back(it->first);
        values.push_back(it->second);
    }
    return reliability_expression.apply(keys, values);
}

/*bool Engine::blacklisted(std::map<std::string,double> &strat) {
    for (std::vector<std::map<std::string,double>>::iterator it = blacklist.begin(); it != blacklist.end(); it++){
        if(*it == strat) return true;
    }

    return false;
}*/

/** **************************************************************
 *                          MONITOR 
/* ***************************************************************
 * ***************************************************************
*/ 
void Engine::monitor() {
    std::cout << "[monitoring]" << std::endl;
    cycles++;

    // request system status data for the knowledge repository
    ros::NodeHandle client_handler;
    ros::ServiceClient client_module;

    client_module = client_handler.serviceClient<archlib::DataAccessRequest>("DataAccessRequest");
    
    //reset the formula
    for (std::map<std::string,double>::iterator it = strategy.begin(); it != strategy.end(); ++it){
        if(it->first.find("CTX_") != std::string::npos)  it->second = 0;
        if(it->first.find("R_") != std::string::npos)  it->second = 1;
        if(it->first.find("F_") != std::string::npos)  it->second = 1;
    }


    archlib::DataAccessRequest r_srv;
    r_srv.request.name = ros::this_node::getName();
    r_srv.request.query = "all:reliability:" + std::to_string(info_quant);

    if(!client_module.call(r_srv)) {
        ROS_ERROR("Failed to connect to data access node.");
        return;
    } /*request reliability for all tasks*/
    
    
    //expecting smth like: "/g3t1_1:success,fail,success;/g4t1:success; ..."
    std::string ans = r_srv.response.content;
    // std::cout << "received=> [" << ans << "]" << std::endl;
    if(ans == ""){
        ROS_ERROR("Received empty answer when asked for reliability.");
    }

    std::vector<std::string> pairs = bsn::utils::split(ans, ';');
    
    for (std::vector<std::string>::iterator it = pairs.begin(); it != pairs.end(); it++) {
        std::vector<std::string> pair = bsn::utils::split(*it, ':');
        std::string first = pair[0];
        std::string second = pair[1];

        //a "/g3t1_1 arrives here"
        std::transform(first.begin(), first.end(),first.begin(), ::toupper); // /G3T1_1
        first.erase(0,1); // G3T1_1
        first.insert(int(first.find('T')),"_"); // G3_T1_1

        std::vector<std::string> values = bsn::utils::split(second, ',');

        strategy["R_"+first] = stod(values[values.size()-1]);
        std::cout << "R_" + first + " = " << strategy["R_" + first] << std::endl;
    } 

    //request context status for all tasks
    archlib::DataAccessRequest c_srv;
    c_srv.request.name = ros::this_node::getName();
    c_srv.request.query = "all:event:1";

    if(!client_module.call(c_srv)) {
        ROS_ERROR("Failed to connect to data access node.");
        return;
    } 
    
    //expecting smth like: "/g3t1_1:activate;/g4t1:deactivate; ..."
    ans = c_srv.response.content;
    //std::cout << "received=> [" << ans << "]" << std::endl;

    if(ans == ""){
        ROS_ERROR("Received empty answer when asked for event.");
    }

    std::vector<std::string> ctx_pairs = bsn::utils::split(ans, ';');
    
    for(std::vector<std::string>::iterator it = ctx_pairs.begin(); it != ctx_pairs.end(); it++){
        std::vector<std::string> pair = bsn::utils::split(*it, ':');
        std::string first = pair[0];
        std::string second = pair[1];

        //a "/g3t1_1 arrives here"
        std::transform(first.begin(), first.end(),first.begin(), ::toupper); // /G3T1_1
        first.erase(0,1); // G3T1_1
        first.insert(int(first.find('T')),"_"); // G3_T1_1

        std::vector<std::string> values = bsn::utils::split(second, ',');
        for (std::vector<std::string>::iterator value = values.begin(); value != values.end(); value++) {
            if(*value=="activate"){
                strategy["CTX_"+first] = 1;
            } else if (*value=="deactivate") {
                strategy["CTX_"+first] = 0;
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
void Engine::analyze() {
    std::cout << "[analyze]" << std::endl;

    //std::cout << "strategy: [";
    //for (std::map<std::string,double>::iterator itt = strategy.begin(); itt != strategy.end(); ++itt) {
    //   std::cout<< itt->first << ":" << itt->second << ", ";
    //}

    double r_curr = calculate_reli();
    std::cout << "current system reliability: " <<  r_curr << std::endl;
    double error = r_ref - r_curr;
    // if the error is out of the stability margin, plan!
    if((error > r_ref*stability_margin) || (error < -stability_margin*r_ref)){
        if(cycles >= monitor_freq/actuation_freq){
            cycles = 0;
            plan();
        }
    }
}

/** **************************************************************
 *                          PLAN
/* ***************************************************************
 * ***************************************************************
*/
void Engine::plan() {
    std::cout << "[plan]" << std::endl;

    std::cout << "r_ref= " << r_ref << std::endl;
    double r_curr = calculate_reli();
    std::cout << "r_curr= " << r_curr << std::endl;
    double error = r_ref - r_curr;
    std::cout << "error= " << error << std::endl;

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    //reset the formula
    std::vector<std::string> r_vec;
    for (std::map<std::string,double>::iterator it = strategy.begin(); it != strategy.end(); ++it) {
        if(it->first.find("R_") != std::string::npos) {
            std::string task = it->first;
            task.erase(0,2);
            if((strategy["CTX_"+task] != 0) && (strategy["F_"+task] != 0)){ // avoid ctx = 0 components thus infinite loops
                r_vec.push_back(it->first);
                strategy[it->first] = r_curr;
            }
        }
    }  

    //reorder r_vec based on priority
    std::vector<std::string> aux = r_vec;
    r_vec.clear();
    std::set<std::pair<std::string,int>, comp> set(priority.begin(), priority.end());
    for(auto const &pair: set){
        if(std::find(aux.begin(), aux.end(), pair.first) != aux.end()){ // key from priority exists in r_vec
            r_vec.push_back(pair.first);
        }
    }

    //print ordered r_vec
    std::cout << "ordered r_vec: [";
    for(std::vector<std::string>::iterator i = r_vec.begin(); i != r_vec.end(); ++i) { 
        std::cout << *i << ", ";
    }
    std::cout << "]" << std::endl;

    // ladies and gentleman, the search...

    std::vector<std::map<std::string, double>> solutions;
    for(std::vector<std::string>::iterator i = r_vec.begin(); i != r_vec.end(); ++i) { 
        //std::cout <<"i: "<< *i << std::endl;

        //reset offset
        for (std::vector<std::string>::iterator it = r_vec.begin(); it != r_vec.end(); ++it) {
            if(error>0){
                strategy[*it] = r_curr*(1-offset);
            } else if(error<0) {
                strategy[*it] = (r_curr*(1+offset)>1)?1:r_curr*(1+offset);
            }
        }
        double r_new = calculate_reli(); // set offset
        std::cout << "offset=" << r_new << std::endl;

        std::map<std::string,double> prev;
        double r_prev=0;
        if(error > 0){
            do {
                prev = strategy;
                r_prev = r_new;
                strategy[*i] += Kp*error;
                r_new = calculate_reli();
            } while(r_new < r_ref && r_prev < r_new && strategy[*i] > 0 && strategy[*i] < 1);
        } else if (error < 0){
            do {
                prev = strategy;
                r_prev = r_new;
                strategy[*i] += Kp*error;
                r_new = calculate_reli();
            } while(r_new > r_ref && r_prev > r_new && strategy[*i] > 0 && strategy[*i] < 1);
        }

        strategy = prev;
        r_new = calculate_reli();

        for(std::vector<std::string>::iterator j = r_vec.begin(); j != r_vec.end(); ++j) { // all the others
            if(*i == *j) continue;
            //std::cout <<"j: "<< *j << std::endl;

            if(error > 0){
                do {
                    prev = strategy;
                    r_prev = r_new;
                    strategy[*j] += Kp*error;
                    r_new = calculate_reli();
                } while(r_new < r_ref && r_prev < r_new && strategy[*j] > 0 && strategy[*j] < 1);
            } else if (error < 0) {
                do {
                    prev = strategy;
                    r_prev = r_new;
                    strategy[*j] += Kp*error;
                    r_new = calculate_reli();
                } while(r_new > r_ref && r_prev > r_new && strategy[*j] > 0 && strategy[*j] < 1);
            }
            
            strategy = prev;
            r_new = calculate_reli();
        }
        solutions.push_back(strategy);
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    uint32_t elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    archlib::Persist msg;

    msg.source = ros::this_node::getName();
    msg.target = "data_access";
    std::string content = std::to_string(Kp) + ";";
    content += std::to_string(offset) + ";";
    content += std::to_string(elapsed_time);
    msg.content = content;
    msg.type = "EngineInfo";
    msg.timestamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    persist_pub.publish(msg);

    for (std::vector<std::map<std::string,double>>::iterator it = solutions.begin(); it != solutions.end(); it++){
        strategy = *it;
        double r_new = calculate_reli();

        std::cout << "strategy: [";
        for (std::map<std::string,double>::iterator itt = (*it).begin(); itt != (*it).end(); ++itt) {
            if(itt->first.find("R_") != std::string::npos) {
                std::cout<< itt->first << ":" << itt->second << ", ";
            }
        }
        std::cout << "] = " << r_new << std::endl;

        if(/*!blacklisted(*it) && */(r_new > r_ref*(1-stability_margin) && r_new < r_ref*(1+stability_margin))){ // if not listed and converges, yay!
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
void Engine::execute() {
    std::cout << "[execute]" << std::endl;

    std::string content = "";
    size_t index = 0;
    bool flag = false;  //nasty
    bool flagO = false; //uber nasty
    //get the reliabilities in the formula and send!
    // send in form "/g3t1_1:0.89;/g4t1=0.2;..."
    for (std::map<std::string,double>::iterator it = strategy.begin(); it != strategy.end(); ++it) {
        std::string aux = "";
        if(it->first.find("R_") != std::string::npos) { //if it is a R_ term of the formula 
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

void Engine::body(){
    ros::NodeHandle n;
    ros::Subscriber t_sub = n.subscribe("exception", 1000, &Engine::receiveException, this);

    ros::Rate loop_rate(rosComponentDescriptor.getFreq());
    while(ros::ok){
        monitor();
        ros::spinOnce();
        loop_rate.sleep();        
    }   

    return;
}
