#include "G3T1_4.hpp"

using namespace bsn::range;
using namespace bsn::resource;
using namespace bsn::generator;
using namespace bsn::operation;
using namespace bsn::configuration;


G3T1_4::G3T1_4(const int32_t &argc, char **argv) :
    type("bloodpressure"),
    battery("bp_batt",100,100,1),
    available(true),
    diasdata_accuracy(1),
    diascomm_accuracy(1),
    systdata_accuracy(1),
    systcomm_accuracy(1),
    active(true),
    params({{"freq",0.90},{"m_avg",5}}),
    filterSystolic(5),
    filterDiastolic(5),
    sensorConfigSystolic(),
    sensorConfigDiastolic(),
    persist(1),
    path("bloodpressure_output.csv") {}

G3T1_4::~G3T1_4() {}

void G3T1_4::setUp() {
    ros::NodeHandle configHandler, n;
    srand(time(NULL));
    std::string s;
    bool b;
    double d;
    Operation op;

    for(int32_t i = 0; i < 2; i++){
        std::vector<std::string> t_probs;
        std::array<float, 25> transitions;
        std::array<bsn::range::Range,5> ranges;
        std::vector<std::string> lrs,mrs,hrs;
        
        std::string x = (i==0)? "syst" : "dias";

        for(uint32_t i = 0; i < transitions.size(); i++){
            for(uint32_t j = 0; j < 5; j++){
                configHandler.getParam(x + "state" + std::to_string(j), s);
                t_probs = op.split(s, ',');
                for(uint32_t k = 0; k < 5; k++){
                    transitions[i++] = std::stod(t_probs[k]);
                }
            }
        }

        { // Configure markov chain
            std::vector<std::string> lrs, mrs, hrs;

            configHandler.getParam(x + "LowRisk", s);
            lrs = op.split(s, ',');
            configHandler.getParam(x + "MidRisk", s);
            mrs = op.split(s, ',');
            configHandler.getParam(x + "HighRisk", s);
            hrs = op.split(s, ',');

            ranges[0] = Range(-1, -1);
            ranges[1] = Range(-1, -1);
            ranges[2] = Range(std::stod(lrs[0]), std::stod(lrs[1]));
            ranges[3] = Range(std::stod(mrs[0]), std::stod(mrs[1]));
            ranges[4] = Range(std::stod(hrs[0]), std::stod(hrs[1]));

            if(i==0){
                markovSystolic = Markov(transitions, ranges, 2);
            } else {
                markovDiastolic = Markov(transitions, ranges, 2);
            }
        }

        { // Configure sensor configuration
            Range low_range = ranges[2];

            std::array<Range, 2> midRanges;
            midRanges[0] = ranges[1];
            midRanges[1] = ranges[3];

            std::array<Range, 2> highRanges;
            highRanges[0] = ranges[0];
            highRanges[1] = ranges[4];

            std::array<Range, 3> percentages;

            configHandler.getParam("lowrisk", s);
            std::vector<std::string> low_p = op.split(s, ',');
            percentages[0] = Range(std::stod(low_p[0]), std::stod(low_p[1]));

            configHandler.getParam("midrisk", s);
            std::vector<std::string> mid_p = op.split(s, ',');
            percentages[1] = Range(std::stod(mid_p[0]), std::stod(mid_p[1]));

            configHandler.getParam("highrisk", s);
            std::vector<std::string> high_p = op.split(s, ',');
            percentages[2] = Range(std::stod(high_p[0]), std::stod(high_p[1]));

            if(i==0){
                sensorConfigSystolic = SensorConfiguration(0,low_range,midRanges,highRanges,percentages);
            } else {
                sensorConfigDiastolic = SensorConfiguration(0,low_range,midRanges,highRanges,percentages);
            }
        }
    }

    { // Configure sensor accuracy
        configHandler.getParam("diasdata_accuracy", d);
        diasdata_accuracy = d / 100;
        configHandler.getParam("diascomm_accuracy", d);
        diascomm_accuracy = d / 100;
        configHandler.getParam("systdata_accuracy", d);
        systdata_accuracy = d / 100;
        configHandler.getParam("systcomm_accuracy", d);
        systcomm_accuracy = d / 100;
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
    
    status_pub =  n.advertise<messages::Status>("collect_status", 10);
    event_pub =  n.advertise<messages::Event>("collect_event", 10);
}

void G3T1_4::tearDown() {
    if (persist)
        fp.close();
}

void G3T1_4::sendStatus(const std::string &id, const double &value) {
    messages::Status msg;

    msg.key = id;
    msg.value = value;

    status_pub.publish(msg);
}

void G3T1_4::sendEvent(const std::string &type, const std::string &description) {
    messages::Event msg;

    msg.type = type;
    msg.description = description;

    event_pub.publish(msg);
}

void G3T1_4::run() {
    double dataS;
    double dataD;
    double risk;
    bool first_exec = true;
    uint32_t id = 0;
    DataGenerator dataGeneratorSys(markovSystolic);
    DataGenerator dataGeneratorDia(markovDiastolic);

    messages::SensorData msgS, msgD;
    msgS.type = "bpms";
    msgD.type = "bpmd";

    ros::NodeHandle n;

    ros::Publisher systolic_pub = n.advertise<messages::SensorData>("systolic_data", 10);
    ros::Publisher diastolic_pub = n.advertise<messages::SensorData>("diastolic_data", 10);

    ros::Rate loop_rate(params["freq"]);

    sendStatus("CTX_G3_T1_4", active?1:0);

    while (ros::ok()) {
        loop_rate = ros::Rate(params["freq"]);        

        /*
        {  // update controller with task info
            sendStatus("CTX_G3_T1_4",1);

            sendStatus("C_G3_T1.411", 0.1);
            sendStatus("R_G3_T1.411", systdata_accuracy);
            sendStatus("F_G3_T1.411", params["freq"]);

            sendStatus("C_G3_T1.412", 0.1);
            sendStatus("R_G3_T1.412", diasdata_accuracy);
            sendStatus("F_G3_T1.412", params["freq"]);

            sendStatus("C_G3_T1.42", 0.1*params["m_avg"]*2);
            sendStatus("R_G3_T1.42", 1);
            sendStatus("F_G3_T1.42", params["freq"]);

            sendStatus("C_G3_T1.43", 0.1*2);
            sendStatus("R_G3_T1.43", (systcomm_accuracy+diascomm_accuracy)/2);
            sendStatus("F_G3_T1.43", params["freq"]);
        }
        */

        { // recharge routine
            //for debugging
           std::cout << "Battery level: " << battery.getCurrentLevel() << "%" <<std::endl;
            if(!active && battery.getCurrentLevel() > 90){
                active = true;
            }
            if(active && battery.getCurrentLevel() < 2){
                active = false;
            }

            sendStatus("CTX_G3_T1_4", active?1:0);
        }

        if (!active) { 
            if(battery.getCurrentLevel() <= 100) battery.generate(2.5);
            continue; 
        }

        { // TASK: Collect bloodpressure data            
            dataS = dataGeneratorSys.getValue();      

            double offset = (1 - systdata_accuracy + (double)rand() / RAND_MAX * (1 - systdata_accuracy)) * dataS;

            if (rand() % 2 == 0)
                dataS = dataS + offset;
            else
                dataS = dataS - offset;

            battery.consume(0.1);

            dataD = dataGeneratorDia.getValue();


            offset = (1 - diasdata_accuracy + (double)rand() / RAND_MAX * (1 - diasdata_accuracy)) * dataD;
            if (rand() % 2 == 0)
                dataD = dataD + offset;
            else
                dataD = dataD - offset;

            battery.consume(0.1);
            
            //for debugging 
            std::cout << std::endl << "New data (systolic): " << dataS <<std::endl;
            std::cout << "New data (diastolic): " << dataD <<std::endl;
        }

        { // TASK: Filter data with moving average
            filterSystolic.setRange(params["m_avg"]);
            filterSystolic.insert(dataS, "bpms");
            dataS = filterSystolic.getValue("bpms");
            battery.consume(0.1*params["m_avg"]);

            filterDiastolic.setRange(params["m_avg"]);
            filterDiastolic.insert(dataD, "bpmd");
            dataD = filterDiastolic.getValue("bpmd");
            battery.consume(0.1*params["m_avg"]);

            
            //for debugging 
            //cout << "Filtered data (systolic): " << dataS <<std::endl;
            //cout << "Filtered data (diastolic): " << dataD <<std::endl;
        }


        { //TASK: Transfer information to CentralHub
            risk = sensorConfigSystolic.evaluateNumber(dataS);
            msgS.data = dataS;
            msgS.risk = risk;
            battery.consume(0.1);
            msgS.batt = battery.getCurrentLevel();
            
            if ((rand() % 100) <= systcomm_accuracy * 100)
                systolic_pub.publish(msgS);

            // for debugging
            std::cout << "Risk: " << risk << "%"  <<std::endl;

            risk = sensorConfigDiastolic.evaluateNumber(dataD);
            msgD.data = dataD;
            msgD.risk = risk;
            battery.consume(0.1);
            msgD.batt = battery.getCurrentLevel();

            if ((rand() % 100) <= diascomm_accuracy * 100)
                diastolic_pub.publish(msgD);

            // for debugging
            std::cout << "Risk: " << risk << "%"  <<std::endl;
        }

        { // Persist sensor data
            if (persist) {
              fp << id++ << ",";
              fp << dataS << ",";
              fp << dataD << ",";
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
