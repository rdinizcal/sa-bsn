#ifndef COMPONENT_DATA_HPP
#define COMPONENT_DATA_HPP

#include <string>
#include <sstream>

class ComponentData {

	public:
    	      ComponentData(/*int &argc, char **argv*/); //Check
    	      virtual ~ComponentData();

            ComponentData(const ComponentData &);
    	      void operator=(const ComponentData &);

            void setTimestamp(double &timestamp);
            double getTimestamp() const;

            void setName(std::string &name);
            std::string getName() const;

            void setType(std::string &type);
            std::string getType() const;

            void setBatteryLevel(double &battery_level);
            double getBatteryLevel() const;

            void setFrequency(double &frequency);
            double getFrequency() const;

            void setCost(double &cost);
            double getCost() const;

            void setRiskStatus(std::string &risk_status);
            std::string getRiskStatus() const;

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