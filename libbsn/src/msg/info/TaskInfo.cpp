#include "msg/info/TaskInfo.hpp"

namespace bsn {
    namespace msg { 
        namespace info {
        
            using namespace std;

            TaskInfo::TaskInfo() : 
                task_id(),
                cost(),
                reliability(),
                frequency() {}
            
            TaskInfo::TaskInfo(const std::string &_task_id, const double &_cost, const double &_reliability, const double &_frequency) : 
                task_id(_task_id),
                cost(_cost),
                reliability(_reliability),
                frequency(_frequency) {}
            
            TaskInfo::~TaskInfo() {}
            
            TaskInfo::TaskInfo(const TaskInfo &obj) :
                SerializableData(),
                task_id(obj.getTask()),
                cost(obj.getCost()),
                reliability(obj.getReliability()),
                frequency(obj.getFrequency()){}
            
            TaskInfo& TaskInfo::operator=(const TaskInfo &obj) {
                task_id = obj.getTask();
                cost = obj.getCost();
                reliability = obj.getReliability();
                frequency = obj.getFrequency();
                return (*this);
            }
            
            int32_t TaskInfo::ID() {
                return 700;
            }
            const string TaskInfo::ShortName() {
                return "TaskInfo";
            }
            const string TaskInfo::LongName() {
                return "info.TaskInfo";
            }
            
            int32_t TaskInfo::getID() const {
                return TaskInfo::ID();
            }
            const string TaskInfo::getShortName() const {
                return TaskInfo::ShortName();
            }
            const string TaskInfo::getLongName() const {
                return TaskInfo::LongName();
            }

            void TaskInfo::setTask(const std::string &_task_id) {
                task_id = _task_id;
            }

            std::string TaskInfo::getTask() const{
                return task_id;
            }

            void TaskInfo::setCost(const double &_cost) {
                cost = _cost;
            }

            double TaskInfo::getCost() const {
                return cost;
            }

            void TaskInfo::setReliability(const double &_reliability){
                reliability = _reliability;
            }

            double TaskInfo::getReliability() const {
                return reliability;
            }

            void TaskInfo::setFrequency(const double &_frequency){
                frequency = _frequency;
            }

            double TaskInfo::getFrequency() const {
                return frequency;
            }

            ostream& TaskInfo::operator<<(ostream &out) const {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Serializer> s = sf.getQueryableNetstringsSerializer(out);
                
                s->write(1, task_id);
                s->write(2, cost);
                s->write(3, reliability);
                s->write(4, frequency);

                return out;
            }
            
            istream& TaskInfo::operator>>(istream &in) {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();
                std::shared_ptr<odcore::serialization::Deserializer> d = sf.getQueryableNetstringsDeserializer(in);
                
                d->read(1, task_id);
                d->read(2, cost);
                d->read(3, reliability);
                d->read(4, frequency);

                return in;
            }
            
            const string TaskInfo::toString() const {
                stringstream sstr;

                sstr << "TaskInfo#" << endl;
                sstr << "Task: " << task_id << endl; 
                sstr << "Cost: " << cost << endl;
                sstr << "Reliability: " << reliability << "%" << endl;
                sstr << "Frequency: " << frequency << "%" << endl;

                return sstr.str();
            }
        }
    }
}