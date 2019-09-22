#include "component/Sensor.hpp"

Sensor::Sensor(int &argc, char **argv, const std::string &name, const std::string &type, const bool &active, const double &noise_factor, const bsn::resource::Battery &battery) : Component(argc, argv, name), type(type), active(active), buffer_size(1), noise_factor(0), battery(battery), data(0.0) {}

Sensor::~Sensor() {}

Sensor& Sensor::operator=(const Sensor &obj) {
    this->type = obj.type;
    this->active = obj.active;
    this->noise_factor = obj.noise_factor;
    this->battery = obj.battery;
    this->data = obj.data;
}

int32_t Sensor::run() {

	setUp();

    ros::NodeHandle nh;
    ros::Subscriber noise_subs = nh.subscribe("uncertainty_"+ros::this_node::getName(), 10, &Sensor::injectUncertainty, this);
    ros::Subscriber reconfig_subs = nh.subscribe("reconfigure_"+ros::this_node::getName(), 10, &Sensor::reconfigure, this);

    sendStatus("init");
    ros::spinOnce();

    while(ros::ok()) {
        ros::Rate loop_rate(rosComponentDescriptor.getFreq());
        ros::spinOnce();

        try{
            body();
        } catch (const std::exception& e) {
            std::cout << "sensor failed: " << e.what() << std::endl;
            sendStatus("fail");
        } 
        loop_rate.sleep();
    }
    
    return 0;
}

void Sensor::body() {
    
    if (!isActive() && battery.getCurrentLevel() > 90){
        turnOn();
    } else if (isActive() && battery.getCurrentLevel() < 2){
        turnOff();        
    }

    if(isActive()) {
        sendStatus("running");
        data = collect();
        apply_noise(data);
        data = process(data);
        transfer(data);
		sendStatus("success");
    } else {
        recharge();
        throw std::domain_error("out of charge");
    }
}

/*
 * error = noise_factor (%)
 * data +- [(error + rand(0,1)) * error] * data
 **/
void Sensor::apply_noise(double &data) {
    double offset = 0;
 
    offset = (noise_factor + ((double)rand() / RAND_MAX) * noise_factor) * data;
    data += (rand()%2==0)?offset:(-1)*offset;
}

void Sensor::reconfigure(const archlib::AdaptationCommand::ConstPtr& msg) {
    std::string action = msg->action.c_str();

    bsn::operation::Operation op;
    std::vector<std::string> pairs = op.split(action, ',');

    for (std::vector<std::string>::iterator it = pairs.begin(); it != pairs.end(); ++it){
        std::vector<std::string> param = op.split(action, '=');

        if(param[0]=="buffer_size"){
            buffer_size = stod(param[1]);
        }
    }
}

void Sensor::injectUncertainty(const archlib::Uncertainty::ConstPtr& msg) {
    std::string content = msg->content;

    bsn::operation::Operation op;
    std::vector<std::string> pairs = op.split(content, ',');

    for (std::vector<std::string>::iterator it = pairs.begin(); it != pairs.end(); ++it){
        std::vector<std::string> param = op.split(content, '=');

        if(param[0]=="noise_factor"){
            noise_factor = stod(param[1]);
        }
    }
}

bool Sensor::isActive() {
    return active;
}

void Sensor::turnOn() {
    active = true;
}

void Sensor::turnOff() {
    active = false;
}

/*  battery will always recover in 20seconds
*
*  b/s = 100% / 20 seconds = 5 %/s 
*      => recovers 5% battery per second
*  if we divide by the execution frequency
*  we get the amount of battery we need to
*  recover per execution cycle to achieve the
*  5 %/s battery recovery rate
*/
void Sensor::recharge() {
    if(battery.getCurrentLevel() <= 100) 
        battery.generate((100/20)/rosComponentDescriptor.getFreq());
}