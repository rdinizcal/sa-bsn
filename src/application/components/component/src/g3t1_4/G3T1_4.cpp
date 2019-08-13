#include "component/g3t1_4/G3T1_4.hpp"

using namespace bsn::range;
using namespace bsn::resource;
using namespace bsn::generator;
using namespace bsn::operation;
using namespace bsn::configuration;


G3T1_4::G3T1_4(const int32_t &argc, char **argv) :
    Sensor(argc, argv, "bloodpressure", true, 1, bsn::resource::Battery("bp_batt", 100, 100, 1)),
    markovSystolic(),
    markovDiastolic(),
    filterSystolic(5),
    filterDiastolic(5),
    sensorConfigSystolic(),
    sensorConfigDiastolic(),
    dias_accuracy(0),
    syst_accuracy(0),
    systolic_data(0),
    diastolic_data(0) {}

G3T1_4::~G3T1_4() {}

void G3T1_4::setUp() {
    srand(time(NULL));
    
    Operation op;

    std::string s;

    for(int32_t i = 0; i < 2; i++){
        std::vector<std::string> t_probs;
        std::array<float, 25> transitions;
        std::array<bsn::range::Range,5> ranges;
        std::vector<std::string> lrs,mrs,hrs;
        
        std::string x = (i==0)? "syst" : "dias";

        for(uint32_t i = 0; i < transitions.size(); i++){
            for(uint32_t j = 0; j < 5; j++){
                handle.getParam(x + "state" + std::to_string(j), s);
                t_probs = op.split(s, ',');
                for(uint32_t k = 0; k < 5; k++){
                    transitions[i++] = std::stod(t_probs[k]);
                }
            }
        }

        { // Configure markov chain
            std::vector<std::string> lrs, mrs, hrs;

            handle.getParam(x + "LowRisk", s);
            lrs = op.split(s, ',');
            handle.getParam(x + "MidRisk", s);
            mrs = op.split(s, ',');
            handle.getParam(x + "HighRisk", s);
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

            handle.getParam("lowrisk", s);
            std::vector<std::string> low_p = op.split(s, ',');
            percentages[0] = Range(std::stod(low_p[0]), std::stod(low_p[1]));

            handle.getParam("midrisk", s);
            std::vector<std::string> mid_p = op.split(s, ',');
            percentages[1] = Range(std::stod(mid_p[0]), std::stod(mid_p[1]));

            handle.getParam("highrisk", s);
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
        double acc;
        handle.getParam("dias_accuracy", acc);
        dias_accuracy = acc / 100;

        handle.getParam("syst_accuracy", acc);
        syst_accuracy = acc / 100;

    }
}

void G3T1_4::tearDown() {}

double G3T1_4::collectSystolic() {
    bsn::generator::DataGenerator dataGenerator(markovSystolic);
    double offset = 0;
    double m_data = 0;

    m_data = dataGenerator.getValue();
    offset = (1 - syst_accuracy + (double)rand() / RAND_MAX * (1 - syst_accuracy)) * m_data;
    m_data += (rand()%2==0)?offset:(-1)*offset;

    battery.consume(0.1);

    ROS_INFO("new data collected: [%s]", std::to_string(m_data).c_str());

    // if rule throw domain_error("failure")
    //if (m_data < 0 || m_data > 100) throw std::domain_error("failure");

    return m_data;
}

double G3T1_4::collectDiastolic() {
    bsn::generator::DataGenerator dataGenerator(markovDiastolic);
    double offset = 0;
    double m_data = 0;

    m_data = dataGenerator.getValue();
    offset = (1 - dias_accuracy + (double)rand() / RAND_MAX * (1 - dias_accuracy)) * m_data;
    m_data += (rand()%2==0)?offset:(-1)*offset;

    battery.consume(0.1);

    ROS_INFO("new data collected: [%s]", std::to_string(m_data).c_str());

    // if rule throw domain_error("failure")
    //if (m_data < 0 || m_data > 100) throw std::domain_error("failure");

    return m_data;
}

double G3T1_4::collect() {

    systolic_data = collectSystolic();
    diastolic_data = collectDiastolic();

    return 0.0;
}

double G3T1_4::processSystolic(const double &m_data) {
    double filtered_data;
    
    filterSystolic.insert(m_data);
    filtered_data = filterSystolic.getValue();
    battery.consume(0.1*filterSystolic.getRange());

    ROS_INFO("filtered data: [%s]", std::to_string(filtered_data).c_str());
    return filtered_data;
}

double G3T1_4::processDiastolic(const double &m_data) {
    double filtered_data;
    
    filterDiastolic.insert(m_data);
    filtered_data = filterDiastolic.getValue();
    battery.consume(0.1*filterDiastolic.getRange());

    ROS_INFO("filtered data: [%s]", std::to_string(filtered_data).c_str());
    return filtered_data;
}

double G3T1_4::process(const double &m_data) {

    systolic_data = processSystolic(systolic_data);
    diastolic_data = processDiastolic(diastolic_data);

    return 0.0;
}

void G3T1_4::transferSystolic(const double &m_data) {
    double risk;
    messages::SensorData msg;
    ros::NodeHandle handle;

    data_pub = handle.advertise<messages::SensorData>("oximeter_data", 10);

    risk = sensorConfigSystolic.evaluateNumber(m_data);
    battery.consume(0.1);

    msg.type = "bpms";
    msg.data = m_data;
    msg.risk = risk;
    msg.batt = battery.getCurrentLevel();

    data_pub.publish(msg);
    
    battery.consume(0.2);

    ROS_INFO("risk calculated and transfered: [%.2f%%]", risk);
}

void G3T1_4::transferDiastolic(const double &m_data) {
    double risk;
    messages::SensorData msg;
    ros::NodeHandle handle;

    data_pub = handle.advertise<messages::SensorData>("oximeter_data", 10);

    risk = sensorConfigDiastolic.evaluateNumber(m_data);
    battery.consume(0.1);

    msg.type = "bpmd";
    msg.data = m_data;
    msg.risk = risk;
    msg.batt = battery.getCurrentLevel();

    data_pub.publish(msg);
    
    battery.consume(0.2);

    ROS_INFO("risk calculated and transfered: [%.2f%%]", risk);
}

void G3T1_4::transfer(const double &m_data) {

    transferSystolic(systolic_data);
    transferDiastolic(diastolic_data);

}