#include "G3T1_1.hpp"

using namespace bsn::range;
using namespace bsn::generator;
using namespace bsn::operation;
using namespace bsn::configuration;

G3T1_1::G3T1_1(const int32_t &argc, char **argv) :
    type("oximeter"),
    battery("oxi_batt", 100, 100, 1),
    available(true),
    data_accuracy(1),
    comm_accuracy(1),
    active(true),
    params({{"freq",0.90},{"m_avg",5}}),
    filter(5),
    persist(1),
    path("oximeter_output.csv") {}

G3T1_1::~G3T1_1() {}

void G3T1_1::setUp() {
    ros::NodeHandle configHandler, n;
    srand(time(NULL));
        
    Operation op;
    
    std::vector<std::string> t_probs;
    std::array<float, 25> transitions;
    std::array<bsn::range::Range,5> ranges;
    std::string s;
    bool b;
    double d;

    for(uint32_t i = 0; i < transitions.size(); i++){
        for(uint32_t j = 0; j < 5; j++){
            configHandler.getParam("state" + std::to_string(j), s);
            t_probs = op.split(s, ',');
            for(uint32_t k = 0; k < 5; k++){
                transitions[i++] = std::stod(t_probs[k]);
            }
        }
    }
    
    { // Configure markov chain
        std::vector<std::string> lrs, mrs, hrs;

        configHandler.getParam("LowRisk", s);
        lrs = op.split(s, ',');
        configHandler.getParam("MidRisk", s);
        mrs = op.split(s, ',');
        configHandler.getParam("HighRisk", s);
        hrs = op.split(s, ',');

        ranges[0] = Range(-1, -1);
        ranges[1] = Range(-1, -1);
        ranges[2] = Range(std::stod(lrs[0]), std::stod(lrs[1]));
        ranges[3] = Range(std::stod(mrs[0]), std::stod(mrs[1]));
        ranges[4] = Range(std::stod(hrs[0]), std::stod(hrs[1]));

        markov = Markov(transitions, ranges, 2);
    }

    { // Configure sensor configuration
        Range low_range = ranges[2];
        
        std::array<Range,2> midRanges;
        midRanges[0] = ranges[1];
        midRanges[1] = ranges[3];
        
        std::array<Range,2> highRanges;
        highRanges[0] = ranges[0];
        highRanges[1] = ranges[4];

        std::array<Range,3> percentages;

        configHandler.getParam("lowrisk", s);
        std::vector<std::string> low_p = op.split(s, ',');
        percentages[0] = Range(std::stod(low_p[0]), std::stod(low_p[1]));

        configHandler.getParam("midrisk", s);
        std::vector<std::string> mid_p = op.split(s, ',');
        percentages[1] = Range(std::stod(mid_p[0]), std::stod(mid_p[1]));

        configHandler.getParam("highrisk", s);
        std::vector<std::string> high_p = op.split(s, ',');
        percentages[2] = Range(std::stod(high_p[0]), std::stod(high_p[1]));

        sensorConfig = SensorConfiguration(0, low_range, midRanges, highRanges, percentages);
    }

 
    { // Configure sensor data_accuracy
        configHandler.getParam("data_accuracy", d);
        data_accuracy = d / 100;
        configHandler.getParam("comm_accuracy", d);
        comm_accuracy = d / 100;
    }

    { // Configure sensor persistency
        configHandler.getParam("persist", b);
        persist = b;
        configHandler.getParam("path", s);
        path = s;

        if (persist) {
            fp.open(path);
            fp << "ID,DATA,RISK,TIME_MS" << std::endl;
        }
    }

    status_pub = n.advertise<messages::Status>("collect_status", 10);
}

void G3T1_1::tearDown() {
    if (persist)
        fp.close();
}

void G3T1_1::sendStatus(const std::string &id, const double &value) {
    messages::Status msg;

    msg.key = id;
    msg.value = value;

    status_pub.publish(msg);
}

void G3T1_1::receiveControlCommand(const messages::ControlCommand::ConstPtr& msg)  {
    active = msg->active;
    double newFreq;
    newFreq = params["freq"] + msg->frequency;
    std::cout << "Frequency changed from " << params["freq"] << " to " << newFreq << std::endl;
    params["freq"] = newFreq;
}

void G3T1_1::run(){
    double data;
    double risk;
    bool first_exec = true;
    uint32_t id = 0;
    bsn::generator::DataGenerator dataGenerator(markov);

    messages::SensorData msg;
    msg.type = "oximeter";
    ros::NodeHandle n;

    dataPub = n.advertise<messages::SensorData>("oximeter_data", 10);
    ros::Subscriber ecgSub = n.subscribe("oximeter_control_command", 10, &G3T1_1::receiveControlCommand, this);

    ros::Rate loop_rate(params["freq"]);

    sendStatus("CTX_G3_T1_1",1);

    while (ros::ok()) {
        loop_rate = ros::Rate(params["freq"]);

        { // update controller with task info            
            sendStatus("CTX_G3_T1_1",1);

            sendStatus("C_G3_T1.11",0.1);
            sendStatus("R_G3_T1.11",data_accuracy);
            sendStatus("F_G3_T1.11",params["freq"]);

            sendStatus("C_G3_T1.12",0.1*params["m_avg"]);
            sendStatus("R_G3_T1.12",1);
            sendStatus("F_G3_T1.12",params["freq"]);

            sendStatus("C_G3_T1.13",0.1);
            sendStatus("R_G3_T1.13",comm_accuracy);
            sendStatus("F_G3_T1.13",params["freq"]);

        }

        { // recharge routine
            //for debugging
            std::cout << "Battery level: " << battery.getCurrentLevel() << "%" << std::endl;
            if(!active && battery.getCurrentLevel() > 90){
                active = true;
            }
            if(active && battery.getCurrentLevel() < 2){
                active = false;
            }
            
            sendStatus("CTX_G3_T1_1", active?1:0);
        }

        if (!active) { 
            if(battery.getCurrentLevel() <= 100) battery.generate(2.5);
            continue; 
        }

        /*
         * Module execution
         */           
           
        { // TASK: Collect oximeter data with data_accuracy
            data = dataGenerator.getValue();
                
            double offset = (1 - data_accuracy + (double)rand() / RAND_MAX * (1 - data_accuracy)) * data;

            if (rand() % 2 == 0)
                data = data + offset;
            else
                data = data - offset;

            battery.consume(0.1);

            //for debugging
            std::cout << "New data: " << data << std::endl << std::endl;
        }

        { // TASK: Filter data with moving average
            filter.setRange(params["m_avg"]);
            filter.insert(data, type);
            data = filter.getValue(type);
            battery.consume(0.1*params["m_avg"]);

            //for debugging
            msg.data = data;
            std::cout << "Filtered data: " << data << std::endl;
        }
        
        { // TASK: Transfer information to CentralHub
            risk = sensorConfig.evaluateNumber(data);
            msg.risk = risk;
            battery.consume(0.1);
            msg.batt = battery.getCurrentLevel();

            if ((rand() % 100) <= comm_accuracy * 100)
                dataPub.publish(msg);
            
            battery.consume(0.1);

            // for debugging
            std::cout << "Risk: " << risk << "%" << std::endl;
        }

        { // Persist sensor data
            if (persist) {
                fp << id++ << ",";
                fp << data << ",";
                fp << risk << ",";
                fp << std::chrono::duration_cast<std::chrono::milliseconds>
                        (std::chrono::time_point_cast<std::chrono::milliseconds>
                        (std::chrono::high_resolution_clock::now()).time_since_epoch()).count() << std::endl;
            }
        }
        ros::spinOnce();

        loop_rate.sleep();
    }

    return tearDown();
}