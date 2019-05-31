#include "BloodpressureModule.hpp"

using namespace bsn::range;
using namespace bsn::resource;
using namespace bsn::generator;
using namespace bsn::operation;
using namespace bsn::configuration;


BloodpressureModule::BloodpressureModule(const int32_t &argc, char **argv) :
    type("bloodpressure"),
    battery("bp_batt",100,100,1),
    available(true),
    diasdata_accuracy(1),
    diascomm_accuracy(1),
    systdata_accuracy(1),
    systcomm_accuracy(1),
    active(true),
    params({{"freq",0.90},{"m_avg",5}}),
    markovSystolic(),
    markovDiastolic(),
    filterSystolic(5),
    filterDiastolic(5),
    sensorConfigSystolic(),
    sensorConfigDiastolic(),
    persist(1),
    path("bloodpressure_output.csv"),
    fp() {}

BloodpressureModule::~BloodpressureModule() {}

void BloodpressureModule::setUp() {
    ros::NodeHandle configHandler;
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
}

void BloodpressureModule::tearDown() {
    if (persist)
        fp.close();
}

void BloodpressureModule::sendTaskInfo(const std::string &task_id, const double &cost, const double &reliability, const double &frequency) {
    // TaskInfo task(task_id, cost, reliability, frequency);
    // Container taskContainer(task);
    // getConference().send(taskContainer);
}

void BloodpressureModule::sendContextInfo(const std::string &context_id, const bool &value) {
    // ContextInfo context(context_id, value, 0, 0, "");
    // Container contextContainer(context);
    // getConference().send(contextContainer);
}

void BloodpressureModule::sendMonitorTaskInfo(const std::string &task_id, const double &cost, const double &reliability, const double &frequency) {
    // MonitorTaskInfo task(task_id, cost, reliability, frequency);
    // Container taskContainer(task);
    // getConference().send(taskContainer);
}

void BloodpressureModule::sendMonitorContextInfo(const std::string &context_id, const bool &value) {
    // MonitorContextInfo context(context_id, value, 0, 0, "");
    // Container contextContainer(context);
    // getConference().send(contextContainer);
}

void BloodpressureModule::run() {
    double dataS;
    double dataD;
    double risk;
    bool first_exec = true;
    uint32_t id = 0;
    DataGenerator dataGeneratorSys(markovSystolic);
    DataGenerator dataGeneratorDia(markovDiastolic);

    messages::SensorData msgS, msgD;
    ros::NodeHandle n;

    ros::Publisher systolic_pub = n.advertise<messages::SensorData>("systolic_data", 10);
    ros::Publisher diastolic_pub = n.advertise<messages::SensorData>("diastolic_data", 10);

    ros::Rate loop_rate(1);

    while (ros::ok()) {
        
        // if(first_exec){ // Send context info warning controller that this sensor is available
        //     sendContextInfo("ABP_available",true);
        //     sendMonitorContextInfo("ABP_available",true);
        //     first_exec = false; 
        // }

        {  // update controller with task info
            /*
            sendContextInfo("ABP_available",true);
            sendTaskInfo("G3_T1.411",0.1,systdata_accuracy,params["freq"]);
            sendTaskInfo("G3_T1.412",0.1,diasdata_accuracy,params["freq"]);
            sendTaskInfo("G3_T1.42",0.1*params["m_avg"]*2,1,params["freq"]);
            sendTaskInfo("G3_T1.43",0.1*2,(systcomm_accuracy+diascomm_accuracy)/2,params["freq"]);
           // and the monitor..
            sendMonitorContextInfo("ABP_available",true);
            sendMonitorTaskInfo("G3_T1.411",0.1,systdata_accuracy,params["freq"]);
            sendMonitorTaskInfo("G3_T1.412",0.1,diasdata_accuracy,params["freq"]);
            sendMonitorTaskInfo("G3_T1.42",0.1*params["m_avg"]*2,1,params["freq"]);
            sendMonitorTaskInfo("G3_T1.43",0.1*2,(systcomm_accuracy+diascomm_accuracy)/2,params["freq"]);
            */
            // sendContextInfo("ABP_available",true);
            // sendTaskInfo("G3_T1.411",0.076,1,1);
            // sendTaskInfo("G3_T1.412",0.076,1,1);
            // sendTaskInfo("G3_T1.42",0.076*params["m_avg"]*2,1,1);
            // sendTaskInfo("G3_T1.43",0.076*2,(1+1)/2,1);
           // and the monitor..
            // sendMonitorContextInfo("ABP_available",true);
            // sendMonitorTaskInfo("G3_T1.411",0.076,1,1);
            // sendMonitorTaskInfo("G3_T1.412",0.076,1,1);
            // sendMonitorTaskInfo("G3_T1.42",0.076*params["m_avg"]*2,1,1);
            // sendMonitorTaskInfo("G3_T1.43",0.076*2,(1+1)/2,1);
        }

        /*{ // recharge routine
            //for debugging
           std::cout << "Battery level: " << battery.getCurrentLevel() << "%" <<std::endl;
            if(!active && battery.getCurrentLevel() > 90){
                active = true;
            }
            if(active && battery.getCurrentLevel() < 2){
                active = false;
            }

            if (rand()%10 > 6) {
                bool x_active = (rand()%2==0)?active:!active;
                sendContextInfo("ABP_available", x_active);
            }

            //sendContextInfo("ABP_available", active);
            sendMonitorContextInfo("ABP_available", active);
        }*/

        // while(!buffer.isEmpty()){ // Receive control command and module update
        //     container = buffer.leave();

        //     active = container.getData<BloodpressureControlCommand>().getActive();
        //     params["freq"] = container.getData<BloodpressureControlCommand>().getFrequency();
        // }

        /*if(!active){ 
            if(battery.getCurrentLevel() <= 100) battery.generate(2.5);
            continue; 
        }*/

        msgS.type = "bpms";
        msgD.type = "bpmd";

        /*
         * Module execution
         */
        if((rand() % 100)+1 < int32_t(params["freq"]*100)){
            
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
                
                if ((rand() % 100) <= systcomm_accuracy * 100)
                    systolic_pub.publish(msgS);
                battery.consume(0.1);

                // for debugging
                std::cout << "Risk: " << risk << "%"  <<std::endl;

                risk = sensorConfigDiastolic.evaluateNumber(dataD);
                msgD.data = dataD;
                msgD.risk = risk;

                if ((rand() % 100) <= diascomm_accuracy * 100)
                    diastolic_pub.publish(msgD);
                battery.consume(0.1);

                // for debugging
                std::cout << "Risk: " << risk << "%"  <<std::endl;
            }

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
