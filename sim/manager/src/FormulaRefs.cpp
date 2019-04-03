#include "FormulaRefs.hpp"

FormulaRefs::FormulaRefs() {}

FormulaRefs::~FormulaRefs(){}

void FormulaRefs::addTask(std::string taskname, double& reli, double& freq, double& cost) {
    task_reliability.insert(std::pair<std::string, double&>(taskname,reli));
    task_frequency.insert(std::pair<std::string, double&>(taskname,freq));
    task_cost.insert(std::pair<std::string, double&>(taskname,cost));
}  


double& FormulaRefs::getReliabilityRef(std::string ref) const {
    return task_reliability.find(ref)->second;
}

double& FormulaRefs::getFrequencyRef(std::string ref) const {
    return task_frequency.find(ref)->second;
}

double& FormulaRefs::getCostRef(std::string ref) const {
    return task_cost.find(ref)->second;
}