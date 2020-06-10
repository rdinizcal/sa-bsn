#include "enactor/Enactor.hpp"
#define W(x) std::cerr << #x << " = " << x << std::endl;

Enactor::Enactor(int  &argc, char **argv, std::string name) : ROSComponent(argc, argv, name), cycles(0), stability_margin(0.02) {}

Enactor::~Enactor() {}

void Enactor::setUp() {
    ros::NodeHandle nh;

    adapt = nh.advertise<archlib::AdaptationCommand>("log_adapt", 10);

    except = nh.advertise<archlib::Exception>("exception", 10);
    enactor_info_server = handle.advertiseService("enactor_info", &Enactor::sendInfo, this);

    double freq;
	nh.getParam("frequency", freq);
    nh.getParam("kp", Kp);
	rosComponentDescriptor.setFreq(freq);

}

void Enactor::tearDown() {}

void Enactor::receiveEvent(const archlib::Event::ConstPtr& msg) {
    if (msg->content=="activate") {
        invocations[msg->source] = {};
        r_curr[msg->source] = 1;
        r_ref[msg->source] = 0.80;
        kp[msg->source] = 200;
        replicate_task[msg->source] = 1;
        freq[msg->source] = 20;
        exception_buffer[msg->source] = 0;

    } else if (msg->content=="deactivate") {
        invocations.erase(msg->source);
        r_curr.erase(msg->source);
        r_ref.erase(msg->source);
        kp.erase(msg->source);
        replicate_task.erase(msg->source);
        freq.erase(msg->source);
        exception_buffer.erase(msg->source);
    }
}

void Enactor::receiveStatus() {
    ros::NodeHandle client_handler;
    ros::ServiceClient client_module;

    client_module = client_handler.serviceClient<archlib::DataAccessRequest>("DataAccessRequest");
    archlib::DataAccessRequest r_srv;
    r_srv.request.name = ros::this_node::getName();
    r_srv.request.query = "all:reliability:";

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

        r_curr[component] = stod(values[values.size() - 1]);
        apply_strategy(component);
    }
}

void Enactor::receiveStrategy(const archlib::Strategy::ConstPtr& msg) {     

    std::vector<std::string> refs = bsn::utils::split(msg->content, ';');

    for(std::vector<std::string>::iterator ref = refs.begin(); ref != refs.end(); ref++){
        std::vector<std::string> pair = bsn::utils::split(*ref, ':'); 
        r_ref[pair[0]] = stod(pair[1]);
    }
}


void Enactor::apply_strategy(const std::string &component) {
    std::cout << "r_ref[" << component << "] = "<< r_ref[component] <<std::endl;
    std::cout << "r_curr[" << component << "] = "<< r_curr[component] <<std::endl;
    std::cout << "kp[" << component << "] = "<< kp[component] <<std::endl;

    double error = r_ref[component] - r_curr[component]; //error = Rref - Rcurr

    if(error > stability_margin*r_ref[component] || error < stability_margin*r_ref[component]) {

        exception_buffer[component] = (exception_buffer[component] < 0) ? 0 : exception_buffer[component] + 1;

        if(component == "/g4t1"){
            // g4t1 reliability is inversely proportional to the sensors frequency
            for (std::map<std::string, double>::iterator it = freq.begin(); it != freq.end(); ++it){
                if(it->first != "/g4t1"){
                    freq[it->first] += (error>0) ? ((-kp[it->first]/100) * error) : ((kp[it->first]/100) * error); 
                    if(freq[(it->first)] <= 0) break;
                    archlib::AdaptationCommand msg;
                    msg.source = ros::this_node::getName();
                    msg.target = it->first;
                    msg.action = "freq=" + std::to_string(freq[(it->first)]);
                    adapt.publish(msg);
                }
            }
            

        } else {
            replicate_task[component] += (error > 0) ? ceil(kp[component] * error) : floor(kp[component] * error);
            if (replicate_task[component] < 1) replicate_task[component] = 1;
            archlib::AdaptationCommand msg;
            msg.source = ros::this_node::getName();
            msg.target = component;
            msg.action = "replicate_collect=" + std::to_string(replicate_task[(component)]);
            adapt.publish(msg);
        }
    } else {
        exception_buffer[component] = (exception_buffer[component] > 0) ? 0 : exception_buffer[component] - 1;
    }

    if(exception_buffer[component]>4){
        archlib::Exception msg;
        msg.source = ros::this_node::getName();
        msg.target = "/engine";
        msg.content = component+"=1";
        except.publish(msg);
        exception_buffer[component] = 0;
    } else if (exception_buffer[component]<-4) {
        archlib::Exception msg;
        msg.source = ros::this_node::getName();
        msg.target = "/engine";
        msg.content = component+"=-1";
        except.publish(msg);
        exception_buffer[component] = 0;
    }
    
    invocations[component].clear();
}

bool Enactor::sendInfo(services::EnactorInfo::Request &req, services::EnactorInfo::Response &res){
    res.kp = std::to_string(Kp);

    return true;
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