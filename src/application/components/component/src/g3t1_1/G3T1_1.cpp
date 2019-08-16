#include "component/g3t1_1/G3T1_1.hpp"

#include <algorithm>
#include <cmath>

using namespace bsn::range;
using namespace bsn::generator;
using namespace bsn::operation;
using namespace bsn::configuration;

G3T1_1::G3T1_1(const int32_t &argc, char **argv) :
    Sensor(argc, argv, "oximeter", true, 1, bsn::resource::Battery("oxi_batt", 100, 100, 1)),
    markov(),
    filter(5),
    sensorConfig() {}

G3T1_1::~G3T1_1() {}

void G3T1_1::setUp() {
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

    { // Configure sensor accuracy
        double acc;
        handle.getParam("accuracy", acc);
        accuracy = acc / 100;
    }
}

void G3T1_1::tearDown() {
}

double G3T1_1::collect() {
    bsn::generator::DataGenerator dataGenerator(markov);
    double offset = 0;
    double m_data = 0;

    m_data = dataGenerator.getValue();
    offset = (1 - accuracy + (double)rand() / RAND_MAX * (1 - accuracy)) * m_data;
    m_data += (rand()%2==0)?offset:(-1)*offset;

    battery.consume(0.1);

    ROS_INFO("new data collected: [%s]", std::to_string(m_data).c_str());

    // if rule throw domain_error("failure")
    if (m_data < 0 || m_data > 100) throw std::domain_error("failure");

    return m_data;
}

bool G3T1_1::check_failure(std::list<double> filter_buffer) {
    std::list<double> min_buffer = filter_buffer;
    std::list<double> max_buffer = filter_buffer;

    double min = 999999;
    double max = -999999;

    double min_avg = 0;
    double max_avg = 0;

    for(std::list<double>::iterator it = filter_buffer.begin(); it != filter_buffer.end() ; it++){
        min = ((*it)<min)?(*it):min;
        max = ((*it)>max)?(*it):max;
    }

    min_buffer.erase(std::remove(min_buffer.begin(), min_buffer.end(), max), min_buffer.end());

    for(std::list<double>::iterator it = min_buffer.begin(); it != min_buffer.end() ; it++){
        min_avg += *it; 
    }

    min_avg /= min_buffer.size();

    max_buffer.erase(std::remove(max_buffer.begin(), max_buffer.end(), min), max_buffer.end());

    for(std::list<double>::iterator it = max_buffer.begin(); it != max_buffer.end() ; it++){
        max_avg += *it; 
    }

    max_avg /= max_buffer.size();

    double limit_diff = filter.getValue()*0.10; // 10% de diferenca!
    double error_max = std::abs(filter.getValue() - max_avg);
    double error_min = std::abs(filter.getValue() - min_avg);

    return (error_max > limit_diff || error_min > limit_diff);
}

double G3T1_1::process(const double &m_data) {
    double filtered_data;
    
    filter.insert(m_data);
    filtered_data = filter.getValue();
    battery.consume(0.1*filter.getRange());

    ROS_INFO("filtered data: [%s]", std::to_string(filtered_data).c_str());
    
    if (check_failure(filter.getBuffer())) throw std::domain_error("failure");

    return filtered_data;
}

void G3T1_1::transfer(const double &m_data) {
    double risk;
    messages::SensorData msg;
    ros::NodeHandle handle;

    data_pub = handle.advertise<messages::SensorData>("oximeter_data", 10);

    risk = sensorConfig.evaluateNumber(m_data);
    battery.consume(0.1);

    msg.type = type;
    msg.data = m_data;
    msg.risk = risk;
    msg.batt = battery.getCurrentLevel();

    data_pub.publish(msg);
    battery.consume(0.2);

    ROS_INFO("risk calculated and transfered: [%.2f%%]", risk);

    if (risk < 0 || risk > 100) throw std::domain_error("failure");
}