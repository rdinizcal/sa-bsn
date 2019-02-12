#include "msg/info/MonitorTaskInfo.hpp"

namespace bsn {
    namespace msg { 
        namespace info {
        
            using namespace std;

            MonitorTaskInfo::MonitorTaskInfo() : 
                task_id(),
                cost(),
                reliability(),
                frequency() {}
            
            MonitorTaskInfo::MonitorTaskInfo(const std::string &_task_id, const double &_cost, const double &_reliability, const double &_frequency) : 
                task_id(_task_id),
                cost(_cost),
                reliability(_reliability),
                frequency(_frequency) {}
            
            MonitorTaskInfo::~MonitorTaskInfo() {}
            
            MonitorTaskInfo::MonitorTaskInfo(const MonitorTaskInfo &obj) :
                SerializableData(),
                task_id(obj.getTask()),
                cost(obj.getCost()),
                reliability(obj.getReliability()),
                frequency(obj.getFrequency()){}
            
            MonitorTaskInfo& MonitorTaskInfo::operator=(const MonitorTaskInfo &obj) {
                task_id = obj.getTask();
                cost = obj.getCost();
                reliability = obj.getReliability();
                frequency = obj.getFrequency();
                return (*this);
            }
            
            int32_t MonitorTaskInfo::ID() {
                return 708;
            }
            const string MonitorTaskInfo::ShortName() {
                return "MonitorTaskInfo";
            }
            const string MonitorTaskInfo::LongName() {
                return "info.MonitorTaskInfo";
            }
            
            int32_t MonitorTaskInfo::getID() const {
                return MonitorTaskInfo::ID();
            }
            const string MonitorTaskInfo::getShortName() const {
                return MonitorTaskInfo::ShortName();
            }
            const string MonitorTaskInfo::getLongName() const {
                return MonitorTaskInfo::LongName();
            }

            void MonitorTaskInfo::setTask(const std::string &_task_id) {
                task_id = _task_id;
            }

            std::string MonitorTaskInfo::getTask() const{
                return task_id;
            }

            void MonitorTaskInfo::setCost(const double &_cost) {
                cost = _cost;
            }

            double MonitorTaskInfo::getCost() const {
                return cost;
            }

            void MonitorTaskInfo::setReliability(const double &_reliability){
                reliability = _reliability;
            }

            double MonitorTaskInfo::getReliability() const {
                return reliability;
            }

            void MonitorTaskInfo::setFrequency(const double &_frequency){
                frequency = _frequency;
            }

            double MonitorTaskInfo::getFrequency() const {
                return frequency;
            }

            ostream& MonitorTaskInfo::operator<<(ostream &out) const {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Serializer> s = sf.getQueryableNetstringsSerializer(out);
                
                s->write(1, task_id);
                s->write(2, cost);
                s->write(3, reliability);
                s->write(4, frequency);

                return out;
            }
            
            istream& MonitorTaskInfo::operator>>(istream &in) {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Deserializer> d = sf.getQueryableNetstringsDeserializer(in);
                
                d->read(1, task_id);
                d->read(2, cost);
                d->read(3, reliability);
                d->read(4, frequency);

                return in;
            }
            
            const string MonitorTaskInfo::toString() const {
                stringstream sstr;

                sstr << "MonitorTaskInfo#" << endl;
                sstr << "Task: " << task_id << endl; 
                sstr << "Cost: " << cost << endl;
                sstr << "Reliability: " << reliability << "%" << endl;
                sstr << "Frequency: " << frequency << "%" << endl;

                return sstr.str();
            }
        }
    }
}