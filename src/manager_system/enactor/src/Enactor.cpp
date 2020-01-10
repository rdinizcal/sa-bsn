#include "enactor/Enactor.hpp"

Enactor::Enactor(int  &argc, char **argv, std::string name) : ROSComponent(argc, argv, name), Kp(200), Ki(10), IW(10), cycles(0), stability_margin(0.02) {}

Enactor::~Enactor() {}

void Enactor::setUp() {
    ros::NodeHandle nh;

    adapt = nh.advertise<archlib::AdaptationCommand>("log_adapt", 10);

    except = nh.advertise<archlib::Exception>("exception", 10);

    double freq;
	nh.getParam("frequency", freq);
	rosComponentDescriptor.setFreq(freq);
    nh.getParam("Kp", Kp);
    nh.getParam("Ki", Ki);
    nh.getParam("IW", IW);
}

void Enactor::tearDown() {}

bool Enactor::sendInfo(services::EnactorInfo::Request &req, services::EnactorInfo::Response &res){
    res.kp = std::to_string(Kp);
    res.ki = std::to_string(Ki);

    return true;
}

void Enactor::receiveEvent(const archlib::Event::ConstPtr& msg) {

    if (msg->content=="activate") {

        invocations[msg->source] = {};
        r_curr[msg->source] = 1;
        r_ref[msg->source] = 0.80;
        //kp[msg->source] = 200;
        replicate_task[msg->source] = 1;
        freq[msg->source] = 20;
        exception_buffer[msg->source] = 0;

        /*
        if(msg->source=="/g3t1_1"){
            r_ref[msg->source] = 0.680054;
        } else if (msg->source=="/g3t1_2") {
            r_ref[msg->source] = 0.606458;
        } else if (msg->source=="/g3t1_3") {
            r_ref[msg->source] = 0.606458;
        } else if (msg->source=="/g3t1_4") {
            r_ref[msg->source] = 1;
        } else if (msg->source=="/g4t1") {
            r_ref[msg->source] = 0.999528;
        }
        */

    } else if (msg->content=="deactivate") {

        invocations.erase(msg->source);
        r_curr.erase(msg->source);
        r_ref.erase(msg->source);
        //kp.erase(msg->source);
        replicate_task.erase(msg->source);
        freq.erase(msg->source);
        exception_buffer.erase(msg->source);

    }
}

void Enactor::receiveStatus(const archlib::Status::ConstPtr& msg) {
    
     if (msg->content=="success") {
        
        if(invocations[msg->source].size() < 50) {
            invocations[msg->source].push_back(1);
        } else {
            invocations[msg->source].pop_front();
            invocations[msg->source].push_back(1);

            int sum = 0;
            for(std::deque<int>::iterator it = invocations[msg->source].begin(); it != invocations[msg->source].end(); ++it) {
                sum += (*it);
            }

            r_curr[msg->source] = double(sum)/double(invocations[msg->source].size());

            apply_strategy(msg->source);
        }

    } else if (msg->content=="fail") {
        //apply strategy only if you have at least 10 invocations information
        if(invocations[msg->source].size() < 50) {
            invocations[msg->source].push_back(0);
        } else {
            invocations[msg->source].pop_front();
            invocations[msg->source].push_back(0);

            int sum = 0;
            for(std::deque<int>::iterator it = invocations[msg->source].begin(); it != invocations[msg->source].end(); ++it) {
                sum += (*it);
            }

            r_curr[msg->source] = double(sum)/double(invocations[msg->source].size());

            apply_strategy(msg->source);
        }
    }
}

void Enactor::receiveStrategy(const archlib::Strategy::ConstPtr& msg) {
    bsn::operation::Operation op;     

    std::vector<std::string> refs = op.split(msg->content, ';');

    for(std::vector<std::string>::iterator ref = refs.begin(); ref != refs.end(); ref++){
        std::vector<std::string> pair = op.split(*ref, ':'); 
        r_ref[pair[0]] = stod(pair[1]);
    }
}


void Enactor::apply_strategy(const std::string &component) {
    //adapt only after at least 60 seconds have passed... (for experiments)
    if(cycles < 60*rosComponentDescriptor.getFreq()) return;

    std::cout << "r_ref[" << component << "] = "<< r_ref[component] <<std::endl;
    std::cout << "r_curr[" << component << "] = "<< r_curr[component] <<std::endl;
    //std::cout << "kp[" << component << "] = "<< kp[component] <<std::endl;
    std::cout << "Kp = " << Kp << std::endl;
    std::cout << "Ki = " << Ki << std::endl;
    
    double error = r_ref[component] - r_curr[component]; //error = Rref - Rcurr
    
    if(error_window[component].size() < IW) {
        error_window[component].push_back(error);
    } else {
        error_window[component].erase(error_window[component].begin());
        error_window[component].push_back(error);
    }

    if(error > stability_margin*r_ref[component] || error < stability_margin*r_ref[component]){

        exception_buffer[component] = (exception_buffer[component] < 0)?0:exception_buffer[component]+1;

        if(component == "/g4t1"){
            // g4t1 reliability is inversely proportional to the sensors frequency
            for (std::map<std::string, double>::iterator it = freq.begin(); it != freq.end(); ++it){
                if(it->first != "/g4t1"){
                    //freq[it->first] += (error>0)?((-kp[it->first]/100) * error):((-kp[it->first]/100) * error);
                    //double error_sum = std::accumulate(error_window[it->first].begin(), error_window[it->first].end(), 0);
                    double error_sum = 0;
                    for(int i=0;i<error_window[it->first].size();i++) {
                        error_sum += error_window[it->first].at(i);
                    }
                    //std::cout << "-----------------------------------------------------------------" << std::endl;
                    //std::cout << "-> Error Sum: " << error_sum << std::endl;
                    /*for(int i=0;i<error_window[component].size();i++) {
                        std::cout << "-> Error " << i << ": " << error_window[component].at(i) << std::endl; 
                    }*/
                    freq[it->first] += (error>0)?((-Kp/100) * error + (-Ki/100) * error_sum):((-Kp/100) * error + (-Ki/100) * error_sum); 
                    //std::cout << "-> (-Kp/100) * error: " << (-Kp/100) * error << std::endl;
                    //std::cout << "-> (-Ki/100) * error_sum: " << (-Ki/100) * error_sum << std::endl;
                    //std::cout << "-----------------------------------------------------------------" << std::endl; 
                    if(freq[(it->first)] <= 0) break;
                    archlib::AdaptationCommand msg;
                    msg.source = ros::this_node::getName();
                    msg.target = it->first;
                    msg.action = "freq=" + std::to_string(freq[(it->first)]);
                    adapt.publish(msg);
                }
            }
            

        } else {
            //replicate_task[component] += (error>0)?ceil(kp[component]*error):floor(kp[component]*error);
            //double error_sum = std::accumulate(error_window[component].begin(), error_window[component].end(), 0);
            double error_sum = 0;
            for(int i=0;i<error_window[component].size();i++) {
                error_sum += error_window[component].at(i);
            }
            //std::cout << "-> Error Sum: " << error_sum << std::endl;
            replicate_task[component] += (error>0)?ceil(Kp*error + Ki*error_sum):floor(Kp*error + Ki*error_sum); //Ki entraria aqui?
            if(replicate_task[component] < 1) replicate_task[component] = 1;
            archlib::AdaptationCommand msg;
            msg.source = ros::this_node::getName();
            msg.target = component;
            msg.action = "replicate_collect=" + std::to_string(replicate_task[(component)]);
            adapt.publish(msg);
            
        }
    } else {
        exception_buffer[component] = (exception_buffer[component] > 0)?0:exception_buffer[component]-1;
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

void Enactor::body(){
    ros::NodeHandle n;

    ros::Subscriber subs_event = n.subscribe("event", 1000, &Enactor::receiveEvent, this);
    ros::Subscriber subs_status = n.subscribe("status", 1000, &Enactor::receiveStatus, this);
    ros::Subscriber subs_strategy = n.subscribe("strategy", 1000, &Enactor::receiveStrategy, this);
    ros::ServiceServer enactor_info_server = handle.advertiseService("enactor_info", &Enactor::sendInfo, this);

    ros::Rate loop_rate(rosComponentDescriptor.getFreq());
    while(ros::ok()){
        if(cycles <= 60*rosComponentDescriptor.getFreq()) ++cycles;
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