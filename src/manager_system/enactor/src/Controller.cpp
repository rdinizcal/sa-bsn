#include "controller/Controller.hpp"

#include <iostream>

Controller::Controller(int &argc, char **argv, std::string name) : Enactor(argc, argv, name) {}

Controller::~Controller() {}

void Controller::setUp() {
    ros::NodeHandle nh;

    adapt = nh.advertise<archlib::AdaptationCommand>("log_adapt", 10);

    except = nh.advertise<archlib::Exception>("exception", 10);

    double freq;
	nh.getParam("frequency", freq);
    nh.getParam("kp", KP);
	rosComponentDescriptor.setFreq(freq);
}

void Controller::apply_strategy(const std::string &component) {
    std::cout << "r_ref[" << component << "] = "<< r_ref[component] <<std::endl;
    std::cout << "r_curr[" << component << "] = "<< r_curr[component] <<std::endl;
    std::cout << "kp[" << component << "] = "<< kp[component] <<std::endl;

    double error = r_ref[component] - r_curr[component]; //error = Rref - Rcurr

    if(error > stability_margin*r_ref[component] || error < stability_margin*r_ref[component]) {

        exception_buffer[component] = (exception_buffer[component] < 0) ? 0 : exception_buffer[component] + 1;

        if(component == "/g4t1"){
            // g4t1 reliability is inversely proportional to the sensors frequency
            /*
                Nota:

                -> Quando o componente é a Centralhub, adaptamos as frequências dos sensores. Porquê?
            */
            /*for (std::map<std::string, double>::iterator it = freq.begin(); it != freq.end(); ++it){
                if(it->first != "/g4t1"){
                    freq[it->first] += (error>0) ? ((-kp[it->first]/100) * error) : ((kp[it->first]/100) * error); 
                    if(freq[(it->first)] <= 0) break;
                    archlib::AdaptationCommand msg;
                    msg.source = ros::this_node::getName();
                    msg.target = it->first;
                    msg.action = "freq=" + std::to_string(freq[(it->first)]);
                    adapt.publish(msg);
                }
            }*/
            //double new_freq = freq[component] + ((error>0) ? ((-kp[component]/100) * error) : ((kp[component]/100) * error)); 
            double new_freq = freq[component] + ((kp[component]/100) * error);
            if(new_freq > 0) {
                freq[component] = new_freq;
                archlib::AdaptationCommand msg;
                msg.source = ros::this_node::getName();
                msg.target = component;
                msg.action = "freq=" + std::to_string(freq[component]);
                adapt.publish(msg);
                /*std::cout << "################################################" << std::endl;
                std::cout << "Adapting Centralhub" << std::endl;
                std::cout << "Action: " << msg.action << std::endl;
                std::cout << "################################################" << std::endl;*/
            } /*else {
                std::cout << "################################################" << std::endl;
                std::cout << "COULD NOT ADAPT CENTRALHUB"<< std::endl;
                std::cout << "Calculated frequency: " << new_freq << std::endl;
                std::cout << "################################################" << std::endl;
            }*/
        } else {
            //freq[component] += (error>0) ? ((-kp[component]/100) * error) : ((kp[component]/100) * error); 
            //double new_freq = freq[component] + ((error>0) ? ((-kp[component]/100) * error) : ((kp[component]/100) * error));
            double new_freq = freq[component] + ((kp[component]/100) * error);
            if(new_freq > 0) {
                freq[component] = new_freq;
                archlib::AdaptationCommand msg;
                msg.source = ros::this_node::getName();
                msg.target = component;
                msg.action = "freq=" + std::to_string(freq[component]);
                adapt.publish(msg);
            }
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