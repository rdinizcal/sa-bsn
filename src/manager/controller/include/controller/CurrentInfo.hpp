#ifndef CURRENT_INFO_HPP
#define CURRENT_INFO_HPP

#include <string>

class CurrentInfo {
    public:
        CurrentInfo(const std::string &module_name, const float &expected_battery_life, 
                        const float &battery_level, const float &cost, const float &frequency, const std::string &risk_status);
        
        CurrentInfo();
        ~CurrentInfo();

        CurrentInfo(const CurrentInfo &);
        CurrentInfo &operator=(const CurrentInfo &);
    
        void setModuleName(const std::string &module_name);
        std::string getModuleName() const;

        void setExpectedBatteryLife(const float &expected_battery_life);
        float getExpectedBatteryLife() const;

        void setBatteryLevel(const float &battery_level);
        float getBatteryLevel() const;

        void setCost(const float &cost);
        float getCost() const;

        void setFrequency(const float &frequency);
        float getFrequency() const;

        void setRiskStatus(const std::string &risk_status);
        std::string getRiskStatus() const;

    private:
        std::string module_name;
        float expected_battery_life;
        float battery_level;
        float cost;
        float frequency;
        std::string risk_status;
        
};

#endif