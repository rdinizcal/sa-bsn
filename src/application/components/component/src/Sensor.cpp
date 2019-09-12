#include "component/Sensor.hpp"


Sensor::Sensor(int &argc, char **argv, const std::string &name, const std::string &type, const bool &active, const double &accuracy, const bsn::resource::Battery &battery) : Component(argc, argv, name), type(type), active(active), accuracy(accuracy), battery(battery), data(0.0) {}

Sensor::~Sensor() {}

Sensor& Sensor::operator=(const Sensor &obj) {
    this->type = obj.type;
    this->active = obj.active;
    this->accuracy = obj.accuracy;
    this->battery = obj.battery;
    this->data = obj.data;
}

int32_t Sensor::run() {

    arch::target_system::Component::setUp();
	setUp();

    sendStatus("init");

    while(ros::ok()) {
        ros::Rate loop_rate(rosComponentDescriptor.getFreq());
        ros::spinOnce();

        try{
            body();
        } catch (const std::exception& e) {
            sendStatus("fail");
        } 
        loop_rate.sleep();
    }
    
    sendStatus("finish");
    tearDown();
    arch::target_system::Component::tearDown();

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
        sendStatus("recharging");
        recharge();
    }
}

void Sensor::apply_noise(double &data) {
    //offset = (1 - accuracy + (double)rand() / RAND_MAX * (1 - accuracy)) * data;
    //data += (rand()%2==0)?offset:(-1)*offset;
}

void Sensor::reconfigure(const archlib::AdaptationCommand::ConstPtr& msg) {
    std::string action = msg->action.c_str();

    char *buffer = strdup(action.c_str());
    char *pair = strtok(buffer, ",");
    char *key = strtok(pair, "=");
    double value  = std::stod(strtok(NULL, "="));

    if(key=="freq"){
        rosComponentDescriptor.setFreq(value);
    }
}

bool Sensor::isActive() {
    return active;
}

void Sensor::turnOn() {
    active = true;
    activate();
}

void Sensor::turnOff() {
    active = false;
    deactivate();
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