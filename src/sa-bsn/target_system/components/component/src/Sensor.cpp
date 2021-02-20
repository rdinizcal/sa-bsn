#include "component/Sensor.hpp"

Sensor::Sensor(int &argc, char **argv, const std::string &name, const std::string &type, const bool &active, const double &noise_factor, const bsn::resource::Battery &battery, const bool &instant_recharge) : Component(argc, argv, name), type(type), active(active), buffer_size(1), replicate_collect(1), noise_factor(0), battery(battery), data(0.0), instant_recharge(instant_recharge), cost(0.0) {}

Sensor::~Sensor() {}

Sensor& Sensor::operator=(const Sensor &obj) {
    this->type = obj.type;
    this->active = obj.active;
    this->noise_factor = obj.noise_factor;
    this->battery = obj.battery;
    this->data = obj.data;
    this->instant_recharge = obj.instant_recharge;
}

int32_t Sensor::run() {

	setUp();

    if (!shouldStart) {
        Component::shutdownComponent();
    }

    ros::NodeHandle nh;
    ros::Subscriber noise_subs = nh.subscribe("uncertainty_"+ros::this_node::getName(), 10, &Sensor::injectUncertainty, this);
    ros::Subscriber reconfig_subs = nh.subscribe("reconfigure_"+ros::this_node::getName(), 10, &Sensor::reconfigure, this);

    sendStatus("init");
    ros::spinOnce();
    
    while (ros::ok()) {
        ros::Rate loop_rate(rosComponentDescriptor.getFreq());
        ros::spinOnce();

        try {
            body();
        } catch (const std::exception& e) {
            std::cout << "sensor failed: " << e.what() << std::endl;
            sendStatus("fail");
            cost = 0;
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

        /*for data replication, as if replicate_collect values were collected*/
        {
            double sum;
            for(int i = 0; i < replicate_collect; ++i) {
                double aux_data = data;
                apply_noise(aux_data);
                sum += aux_data;
            }
            data = sum/replicate_collect;
        }

        data = process(data);
        transfer(data);
		sendStatus("success");
        sendEnergyStatus(cost);
        cost = 0.0;
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
    noise_factor = 0;
}

void Sensor::reconfigure(const archlib::AdaptationCommand::ConstPtr& msg) {
    std::string action = msg->action.c_str();

    std::vector<std::string> pairs = bsn::utils::split(action, ',');

    for (std::vector<std::string>::iterator it = pairs.begin(); it != pairs.end(); ++it){
        std::vector<std::string> param = bsn::utils::split(action, '=');

        if(param[0]=="freq"){
            double new_freq =  stod(param[1]);
            rosComponentDescriptor.setFreq(new_freq);
        } else if (param[0]=="replicate_collect") {
            int new_replicate_collect = stoi(param[1]);
            if(new_replicate_collect>1 && new_replicate_collect<200) replicate_collect = new_replicate_collect;
        }
    }
}

void Sensor::injectUncertainty(const archlib::Uncertainty::ConstPtr& msg) {
    std::string content = msg->content;

    std::vector<std::string> pairs = bsn::utils::split(content, ',');

    for (std::vector<std::string>::iterator it = pairs.begin(); it != pairs.end(); ++it){
        std::vector<std::string> param = bsn::utils::split(content, '=');

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

/*  battery will always recover in 200seconds
*
*  b/s = 100% / 200 seconds = 0.2 %/s 
*      => recovers 5% battery per second
*  if we divide by the execution frequency
*  we get the amount of battery we need to
*  recover per execution cycle to achieve the
*  0.2 %/s battery recovery rate
*/
void Sensor::recharge() {
    if(!instant_recharge) {
        if(battery.getCurrentLevel() <= 100) {
            battery.generate(1);
            // battery.generate((100/2000)/rosComponentDescriptor.getFreq());
        }
    } else {
        battery.generate(100);
    }
}