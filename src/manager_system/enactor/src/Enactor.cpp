#include "enactor/Enactor.hpp"
#define W(x) std::cerr << #x << " = " << x << std::endl;

Enactor::Enactor(int &argc, char **argv, std::string name) : ROSComponent(argc, argv, name), cycles(0), stability_margin(0.02) {}

Enactor::~Enactor() {}

void Enactor::tearDown() {}

void Enactor::receiveAdaptationParameter() {
    ros::NodeHandle client_handler;
    ros::ServiceClient client_module;

    client_module = client_handler.serviceClient<archlib::EngineRequest>("EngineRequest");
    archlib::EngineRequest adapt_srv;
    
    if(!client_module.call(adapt_srv)) {
        ROS_ERROR("Failed to connect to Strategy Manager node.");
        return;
    }

    adaptation_parameter = adapt_srv.response.content;

    if(adaptation_parameter != "reliability" && adaptation_parameter != "cost") {
        ROS_ERROR("Invalid adaptation parameter received.");
        return;
    }
}

void Enactor::receiveStatus() {
    ros::NodeHandle client_handler;
    ros::ServiceClient client_module;

    client_module = client_handler.serviceClient<archlib::DataAccessRequest>("DataAccessRequest");
    archlib::DataAccessRequest r_srv;
    r_srv.request.name = ros::this_node::getName();
    if(adaptation_parameter == "reliability") {
        r_srv.request.query = "all:reliability:";
    } else {
        r_srv.request.query = "all:cost:";
    }

    if (!client_module.call(r_srv)) {
        ROS_ERROR("Failed to connect to data access node.");
        return;
    }
    
    std::string ans = r_srv.response.content;
    // std::cout << "received=> [" << ans << "]" << std::endl;
    if (ans == "") {
        ROS_ERROR("Received empty answer when asked for status.");
    }

    std::vector<std::string> pairs = bsn::utils::split(ans, ';');

    for (auto s : pairs) {
        std::vector<std::string> pair = bsn::utils::split(s, ':');
        std::string component = pair[0];
        std::string content = pair[1];

        std::vector<std::string> values = bsn::utils::split(content, ',');

        if(adaptation_parameter == "reliability") {
            r_curr[component] = stod(values[values.size() - 1]);
            apply_reli_strategy(component);
        } else {
            c_curr[component] = stod(values[values.size() - 1]);
            apply_cost_strategy(component);
        }
    }
}

void Enactor::receiveStrategy(const archlib::Strategy::ConstPtr& msg) {     

    std::vector<std::string> refs = bsn::utils::split(msg->content, ';');

    for(std::vector<std::string>::iterator ref = refs.begin(); ref != refs.end(); ref++){
        std::vector<std::string> pair = bsn::utils::split(*ref, ':'); 
        if(adaptation_parameter == "reliability") {
            r_ref[pair[0]] = stod(pair[1]);
        } else {
            c_ref[pair[0]] = stod(pair[1]);
        }
    }
}

void Enactor::body(){
    ros::NodeHandle n;

    ros::Subscriber subs_event = n.subscribe("event", 1000, &Enactor::receiveEvent, this);
    ros::Subscriber subs_strategy = n.subscribe("strategy", 1000, &Enactor::receiveStrategy, this);

    ros::Rate loop_rate(rosComponentDescriptor.getFreq());
    while(ros::ok()){
        if(cycles <= 60*rosComponentDescriptor.getFreq()) ++cycles;
        receiveStatus();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void Enactor::print() {
    /*
    for(std::vector<std::string>::iterator it = connected.begin(); it != connected.end(); ++it) {
        std::cout << "****************************************" << std::endl;
        std::cout << *it << std::endl;
        std::cout << "  - invocations: [";
        for(std::deque<int>::iterator itt = invocations[*it].begin(); itt != invocations[*it].end(); ++itt) std::cout << *itt << " ";
        std::cout << "]" << std::endl;
        std::cout << "  - r curr: " << r_curr[*it] << std::endl;
        std::cout << "  - buffer size: " << replicate_task[*it] << std::endl;
        std::cout << "****************************************" << std::endl;
    }
    */
}