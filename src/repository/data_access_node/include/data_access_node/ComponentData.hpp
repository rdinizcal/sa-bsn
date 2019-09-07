#ifndef COMPONENT_DATA_HPP
#define COMPONENT_DATA_HPP

#include <string>
#include <sstream>

class ComponentData {

	public:
    	      ComponentData(/*int &argc, char **argv*/); //Check
    	      virtual ~ComponentData();

            ComponentData(const ComponentData &);
    	      ComponentData &operator=(const ComponentData &);

            void setTimestamp(double &timestamp);
            double getTimestamp();

            void setName(std::string &name);
            std::string getName();

            void setType(std::string &type);
            std::string getType();

            void setBatteryLevel(double &battery_level);
            double getBatteryLevel();

            void setFrequency(double &frequency);
            double getFrequency();

            void setCost(double &cost);
            double getCost();

            void setRiskStatus(std::string &risk_status);
            std::string getRiskStatus();

            std::string toString();

  	private:
        double timestamp;
        std::string name;
        std::string type;
        double battery_level;
        double frequency;
        double cost;
        std::string risk_status;
};

#endif 