#include "component/g3t1_1/G3T1_1.hpp"

#define BATT_UNIT 0.1

#include <algorithm>
#include <cmath>

using namespace bsn::range;
using namespace bsn::generator;
using namespace bsn::operation;
using namespace bsn::configuration;

G3T1_1::G3T1_1(int &argc, char **argv, const std::string &name) :
    Sensor(argc, argv, name, "oximeter", true, 1, bsn::resource::Battery("oxi_batt", 100, 100, 1)),
    markov(),
    dataGenerator(),
    filter(1),
    sensorConfig(),
    collected_risk() {}

G3T1_1::~G3T1_1() {}

void G3T1_1::setUp() {
    Component::setUp();

    srand(time(NULL));
        
    Operation op;
    
    std::vector<std::string> t_probs;
    std::array<float, 25> transitions;
    std::array<bsn::range::Range,5> ranges;
    std::string s;

    for(uint32_t i = 0; i < transitions.size(); i++){
        for(uint32_t j = 0; j < 5; j++){
            handle.getParam("state" + std::to_string(j), s);
            t_probs = op.split(s, ',');
            for(uint32_t k = 0; k < 5; k++){
                transitions[i++] = std::stod(t_probs[k]);
            }
        }
    }
    
    { // Configure markov chain
        std::vector<std::string> lrs, mrs, hrs;

        handle.getParam("LowRisk", s);
        lrs = op.split(s, ',');
        handle.getParam("MidRisk", s);
        mrs = op.split(s, ',');
        handle.getParam("HighRisk", s);
        hrs = op.split(s, ',');

        ranges[0] = Range(-1, -1);
        ranges[1] = Range(-1, -1);
        ranges[2] = Range(std::stod(lrs[0]), std::stod(lrs[1]));
        ranges[3] = Range(std::stod(mrs[0]), std::stod(mrs[1]));
        ranges[4] = Range(std::stod(hrs[0]), std::stod(hrs[1]));

        markov = Markov(transitions, ranges, 2);
        bsn::generator::DataGenerator dt(markov);
        dataGenerator = dt;
        dataGenerator.setSeed();
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

        handle.getParam("lowrisk", s);
        std::vector<std::string> low_p = op.split(s, ',');
        percentages[0] = Range(std::stod(low_p[0]), std::stod(low_p[1]));

        handle.getParam("midrisk", s);
        std::vector<std::string> mid_p = op.split(s, ',');
        percentages[1] = Range(std::stod(mid_p[0]), std::stod(mid_p[1]));

        handle.getParam("highrisk", s);
        std::vector<std::string> high_p = op.split(s, ',');
        percentages[2] = Range(std::stod(high_p[0]), std::stod(high_p[1]));

        sensorConfig = SensorConfiguration(0, low_range, midRanges, highRanges, percentages);
    }

}

void G3T1_1::tearDown() {
    Component::tearDown();
}

double G3T1_1::collect() {
    double m_data = 0;
    m_data = dataGenerator.getValue();

    //battery.consume(BATT_UNIT);
    ROS_INFO("new data collected: [%s]", std::to_string(m_data).c_str());

    collected_risk = sensorConfig.evaluateNumber(m_data);

    return m_data;
}

double G3T1_1::process(const double &m_data) {
    double filtered_data;
    
    filter.insert(m_data);
    filtered_data = filter.getValue();
    //battery.consume(BATT_UNIT*filter.getRange());

    ROS_INFO("filtered data: [%s]", std::to_string(filtered_data).c_str());
    return filtered_data;
}

void G3T1_1::transfer(const double &m_data) {
    double risk;
    risk = sensorConfig.evaluateNumber(m_data);
    //battery.consume(BATT_UNIT);
    if (risk < 0 || risk > 100) throw std::domain_error("risk data out of boundaries");
    if (label(risk) != label(collected_risk)) throw std::domain_error("sensor accuracy fail");

    ros::NodeHandle handle;
    data_pub = handle.advertise<messages::SensorData>("oximeter_data", 10);
    messages::SensorData msg;
    msg.type = type;
    msg.data = m_data;
    msg.risk = risk;
    msg.batt = battery.getCurrentLevel();

    data_pub.publish(msg);
    //battery.consume(0.2);

    ROS_INFO("risk calculated and transferred: [%.2f%%]", risk);
}

std::string G3T1_1::label(double &risk) {
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