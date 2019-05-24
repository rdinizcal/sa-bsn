#include "ECGModule.hpp"

using namespace bsn::range;
using namespace bsn::generator;
using namespace bsn::operation;
using namespace bsn::configuration;

ECGModule::ECGModule(const int32_t &argc, char **argv) :
    type("ecg"),
    battery("ecg_batt", 100, 100, 1),
    available(true),
    data_accuracy(1),
    comm_accuracy(1),
    active(true),
    params({{"freq",0.9},{"m_avg",5}}),
    filter(5),
    sensorConfig(),
    persist(1),
    path("ecg_output.csv") {}

ECGModule::~ECGModule() {}

void ECGModule::setUp() {
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
        std::vector<std::string> lrs,mrs0,hrs0,mrs1,hrs1;

        configHandler.getParam("LowRisk", s);
        lrs = op.split(s, ',');
        configHandler.getParam("MidRisk0", s);
        mrs0 = op.split(s, ',');
        configHandler.getParam("HighRisk0", s);
        hrs0 = op.split(s, ',');
        configHandler.getParam("MidRisk1", s);
        mrs1 = op.split(s, ',');
        configHandler.getParam("HighRisk1", s);
        hrs1 = op.split(s, ',');

        ranges[0] = Range(std::stod(hrs0[0]), std::stod(hrs0[1]));
        ranges[1] = Range(std::stod(mrs0[0]), std::stod(mrs0[1]));
        ranges[2] = Range(std::stod(lrs[0]), std::stod(lrs[1]));
        ranges[3] = Range(std::stod(mrs1[0]), std::stod(mrs1[1]));
        ranges[4] = Range(std::stod(hrs1[0]), std::stod(hrs1[1]));

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

    taskPub =  n.advertise<bsn::TaskInfo>("task_info", 10);
    contextPub =  n.advertise<bsn::ContextInfo>("context_info", 10);
}

void ECGModule::tearDown() {
    if (persist)
        fp.close();
}

void ECGModule::sendTaskInfo(const std::string &task_id, const double &cost, const double &reliability, const double &frequency) {
    bsn::TaskInfo msg;

    msg.task_id = task_id;
    msg.cost = cost;
    msg.reliability = reliability;
    msg.frequency = frequency;
    taskPub.publish(msg);
}

void ECGModule::sendContextInfo(const std::string &context_id, const bool &status) {
    bsn::ContextInfo msg;

    msg.context_id = context_id;
    msg.status = status;
    contextPub.publish(msg);
}

void ECGModule::receiveControlCommand(const bsn::ControlCommand::ConstPtr& msg)  {
    active = msg->active;
    params["freq"] = msg->frequency;
}

void ECGModule::run() {
    double data;
    double risk;
    bool first_exec = true;
    uint32_t id = 0;
    bsn::generator::DataGenerator dataGenerator(markov);

    bsn::SensorData msg;
    msg.type = "ecg";

    ros::NodeHandle n;

    dataPub = n.advertise<bsn::SensorData>("ecg_data", 10);
    ros::Subscriber ecgSub = n.subscribe("ecg_control_command", 10, &ECGModule::receiveControlCommand, this);

    ros::Rate loop_rate(params["freq"]);

    sendContextInfo("ECG_available",true);    

    while (ros::ok()) {
        { // update controller with task info            
            sendContextInfo("ECG_available",true);
            sendTaskInfo("G3_T1.31",0.1,data_accuracy,params["freq"]);
            sendTaskInfo("G3_T1.32",0.1*params["m_avg"],1,params["freq"]);
            sendTaskInfo("G3_T1.33",0.1,comm_accuracy,params["freq"]);
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
            
            if (rand()%10 > 6) {
                    bool x_active = (rand()%2==0)?active:!active;
                    sendContextInfo("ECG_available", x_active);
            }
            sendContextInfo("ECG_available", active);
        }

        if(!active){ 
            if(battery.getCurrentLevel() <= 100) battery.generate(2.5);
            continue; 
        }

        /*
         * Module execution
         */
        if((rand() % 100)+1 < int32_t(params["freq"]*100)){
           
            { // TASK: Collect thermometer data with data_accuracy
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

                // for debugging
                std::cout << "Risk: " << risk << "%" << std::endl;
            }
            
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
