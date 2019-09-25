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
    dataGeneratorSystolic(),
    markovDiastolic(),
    dataGeneratorDiastolic(),
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
        std::array<bsn::range::Range,5> ranges;
        
        std::string x = (i==0)? "syst" : "dias";

        { // Get risks for
            std::vector<std::string> lrs,mrs0,hrs0,mrs1,hrs1;

            handle.getParam(x + "LowRisk", s);
            lrs = op.split(s, ',');
            handle.getParam(x + "MidRisk0", s);
            mrs0 = op.split(s, ',');
            handle.getParam(x + "HighRisk0", s);
            hrs0 = op.split(s, ',');
            handle.getParam(x + "MidRisk1", s);
            mrs1 = op.split(s, ',');
            handle.getParam(x + "HighRisk1", s);
            hrs1 = op.split(s, ',');

            ranges[0] = Range(std::stod(hrs0[0]), std::stod(hrs0[1]));
            ranges[1] = Range(std::stod(mrs0[0]), std::stod(mrs0[1]));
            ranges[2] = Range(std::stod(lrs[0]), std::stod(lrs[1]));
            ranges[3] = Range(std::stod(mrs1[0]), std::stod(mrs1[1]));
            ranges[4] = Range(std::stod(hrs1[0]), std::stod(hrs1[1]));
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
    double m_data = 0;
    ros::ServiceClient client = handle.serviceClient<services::PatientData>("getPatientData");
    services::PatientData srv;

    srv.request.vitalSign = "systolic_pressure";

    if (client.call(srv)) {
        m_data = srv.response.data;
        ROS_INFO("new data collected: [%s]", std::to_string(m_data).c_str());
    } else {
        ROS_INFO("error collecting data");
    }
    //battery.consume(BATT_UNIT);

    collected_systolic_risk = sensorConfigSystolic.evaluateNumber(m_data);

    return m_data;
}

double G3T1_4::collectDiastolic() {
    double m_data = 0;
    ros::ServiceClient client = handle.serviceClient<services::PatientData>("getPatientData");
    services::PatientData srv;

    srv.request.vitalSign = "diastolic_pressure";

    if (client.call(srv)) {
        m_data = srv.response.data;
        ROS_INFO("new data collected: [%s]", std::to_string(m_data).c_str());
    } else {
        ROS_INFO("error collecting data");
    }
    //battery.consume(BATT_UNIT);

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
    if (label(sensorConfigSystolic, risk) != label(sensorConfigSystolic, collected_systolic_risk)) throw std::domain_error("sensor accuracy fail");

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
    if (label(sensorConfigDiastolic, risk) != label(sensorConfigDiastolic, collected_diastolic_risk)) throw std::domain_error("sensor accuracy fail");

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