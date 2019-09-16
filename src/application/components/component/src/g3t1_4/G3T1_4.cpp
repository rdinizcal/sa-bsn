#include "component/g3t1_4/G3T1_4.hpp"

#define BATT_UNIT 0.1

using namespace bsn::range;
using namespace bsn::resource;
using namespace bsn::generator;
using namespace bsn::operation;
using namespace bsn::configuration;


G3T1_4::G3T1_4(int &argc, char **argv, const std::string &name) :
    Sensor(argc, argv, name, "bloodpressure", true, 1, bsn::resource::Battery("bp_batt", 100, 100, 1)),
    markovSystolic(),
    markovDiastolic(),
    filterSystolic(1),
    filterDiastolic(1),
    sensorConfigSystolic(),
    sensorConfigDiastolic(),
    systolic_data(0),
    diastolic_data(0),
    collected_systolic_risk(),
    collected_diastolic_risk() {}

G3T1_4::~G3T1_4() {}

void G3T1_4::setUp() {
    Component::setUp();
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

}

void G3T1_4::tearDown() {
    Component::tearDown();
}

double G3T1_4::collectSystolic() {
    bsn::generator::DataGenerator dataGenerator(markovSystolic);
    double m_data = 0;

    m_data = dataGenerator.getValue();
    //battery.consume(BATT_UNIT);

    ROS_INFO("new data collected: [%s]", std::to_string(m_data).c_str());

    collected_systolic_risk = sensorConfigSystolic.evaluateNumber(m_data);

    return m_data;
}

double G3T1_4::collectDiastolic() {
    bsn::generator::DataGenerator dataGenerator(markovDiastolic);
    double m_data = 0;

    m_data = dataGenerator.getValue();
    //battery.consume(BATT_UNIT);

    ROS_INFO("new data collected: [%s]", std::to_string(m_data).c_str());

    collected_diastolic_risk = sensorConfigDiastolic.evaluateNumber(m_data);

    return m_data;
}

double G3T1_4::collect() {

    systolic_data = collectSystolic();
    apply_noise(systolic_data);

    diastolic_data = collectSystolic();
    apply_noise(diastolic_data);

    return 0.0;
}

double G3T1_4::processSystolic(const double &m_data) {
    double filtered_data;
    
    filterSystolic.insert(m_data);
    filtered_data = filterSystolic.getValue();
    //battery.consume(BATT_UNIT*filterSystolic.getRange());

    ROS_INFO("filtered data: [%s]", std::to_string(filtered_data).c_str());
    return filtered_data;
}

double G3T1_4::processDiastolic(const double &m_data) {
    double filtered_data;
    
    filterDiastolic.insert(m_data);
    filtered_data = filterDiastolic.getValue();
    //battery.consume(BATT_UNIT*filterDiastolic.getRange());

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
    risk = sensorConfigSystolic.evaluateNumber(m_data);
    //battery.consume(BATT_UNIT);
    if (risk < 0 || risk > 100) throw std::domain_error("risk data out of boundaries");
    if (label(sensorConfigSystolic, risk) != label(sensorConfigSystolic, collected_systolic_risk)) throw std::domain_error("content failure due to noise");

    ros::NodeHandle handle;
    data_pub = handle.advertise<messages::SensorData>("systolic_data", 10);
    messages::SensorData msg;
    msg.type = "bpms";
    msg.data = m_data;
    msg.risk = risk;
    msg.batt = battery.getCurrentLevel();
    data_pub.publish(msg);
    //battery.consume(0.2);

    ROS_INFO("risk calculated and transferred: [%.2f%%]", risk);
    
}

void G3T1_4::transferDiastolic(const double &m_data) {
    double risk;
    risk = sensorConfigDiastolic.evaluateNumber(m_data);
    //battery.consume(BATT_UNIT);
    if (risk < 0 || risk > 100) throw std::domain_error("risk data out of boundaries");
    if (label(sensorConfigDiastolic, risk) != label(sensorConfigDiastolic, collected_diastolic_risk)) throw std::domain_error("content failure due to noise");

    ros::NodeHandle handle;
    data_pub = handle.advertise<messages::SensorData>("diastolic_data", 10);
    messages::SensorData msg;
    msg.type = "bpmd";
    msg.data = m_data;
    msg.risk = risk;
    msg.batt = battery.getCurrentLevel();
    data_pub.publish(msg);
    //battery.consume(0.2);

    ROS_INFO("risk calculated and transferred: [%.2f%%]", risk);

}

void G3T1_4::transfer(const double &m_data) {

    transferSystolic(systolic_data);
    transferDiastolic(diastolic_data);

}

std::string G3T1_4::label(bsn::configuration::SensorConfiguration &sensorConfig, double &risk) {
    std::string ans;
    if(sensorConfig.isLowRisk(risk)){
        ans = "low";
    } else if (sensorConfig.isMediumRisk(risk)) {
        ans = "moderate";
    } else if (sensorConfig.isHighRisk(risk)) {
        ans = "high";
    } else {
        ans = "unknown";
    }

    return ans;
}