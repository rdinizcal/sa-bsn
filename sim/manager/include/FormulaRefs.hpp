#ifndef MANAGER_FORMULAREFS_HPP
#define MANAGER_FORMULAREFS_HPP

#include <string>
#include <map>

class FormulaRefs {

    public:
        FormulaRefs();
        ~FormulaRefs();

        void addTask(std::string /*taskname*/, double& /*reli*/, double& /*freq*/, double& /*cost*/);
        void addTask(std::string /*taskname*/, double& /*reli*/, double& /*freq*/);

        double& getReliabilityRef(std::string /*ref*/) const;
        double& getFrequencyRef(std::string /*ref*/) const;
        double& getCostRef(std::string /*ref*/) const;

    private:
        std::map<std::string,double&> task_reliability;
        std::map<std::string,double&> task_frequency;
        std::map<std::string,double&> task_cost;
        
};

#endif