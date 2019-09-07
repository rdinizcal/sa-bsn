#include "component/g3t1_2/G3T1_2.hpp"

using namespace bsn::range;
using namespace bsn::generator;
using namespace bsn::operation;
using namespace bsn::configuration;

G3T1_2::G3T1_2(const int32_t &argc, char **argv) :
    Sensor(argc, argv, "ecg", true, 1, bsn::resource::Battery("ecg_batt", 100, 100, 1)),
    markov(),
    filter(5),
    sensorConfig() {}

G3T1_2::~G3T1_2() {}

void G3T1_2::setUp() {
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
        std::vector<std::string> lrs,mrs0,hrs0,mrs1,hrs1;

        handle.getParam("LowRisk", s);
        lrs = op.split(s, ',');
        handle.getParam("MidRisk0", s);
        mrs0 = op.split(s, ',');
        handle.getParam("HighRisk0", s);
        hrs0 = op.split(s, ',');
        handle.getParam("MidRisk1", s);
        mrs1 = op.split(s, ',');
        handle.getParam("HighRisk1", s);
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

void G3T1_2::tearDown() {
}

double G3T1_2::collect() {
    bsn::generator::DataGenerator dataGenerator(markov);
    double offset = 0;
    double m_data = 0;

    m_data = dataGenerator.getValue();
    offset = (1 - accuracy + (double)rand() / RAND_MAX * (1 - accuracy)) * m_data;
    m_data += (rand()%2==0)?offset:(-1)*offset;

    battery.consume(0.1);

    ROS_INFO("new data collected: [%s]", std::to_string(m_data).c_str());

    // if rule throw domain_error("failure")
    //if (m_data < 0 || m_data > 100) throw std::domain_error("failure");

    return m_data;
}

double G3T1_2::process(const double &m_data) {
    double filtered_data;
    
    filter.insert(m_data, type);
    filtered_data = filter.getValue(type);
    battery.consume(0.1*filter.getRange());

    ROS_INFO("filtered data: [%s]", std::to_string(filtered_data).c_str());
    return filtered_data;
}

void G3T1_2::transfer(const double &m_data) {
    double risk;
    messages::SensorData msg;
    ros::NodeHandle handle;

    data_pub = handle.advertise<messages::SensorData>("oximeter_data", 10);
    info_pub = handle.advertise<messages::Info>("collect_info", 1000);

    risk = sensorConfig.evaluateNumber(m_data);
    battery.consume(0.1);

    msg.type = type;
    msg.data = m_data;
    msg.risk = risk;
    msg.batt = battery.getCurrentLevel();

    data_pub.publish(msg);
    
    battery.consume(0.2);

    messages::Info infoMsg;
    std::string content = "";
    content += "timestamp:,";
    content += "name:"+moduleDescriptor.getName()+",";
    content += "type:"+type+",";
    content += "battery:"+std::to_string(battery.getCurrentLevel())+",";
    content += "frequency:"+std::to_string(moduleDescriptor.getFreq())+",";
    content += "cost:"+std::to_string((0.1 + 0.1*filter.getRange() + 0.1 + 0.2))+",";
    content += "risk:"+std::to_string(risk); //Error!;

    infoMsg.source = moduleDescriptor.getName();
    infoMsg.target = "/repository";
    infoMsg.content = content;

    info_pub.publish(infoMsg);

    ROS_INFO("risk calculated and transfered: [%.2f%%]", risk);
}
