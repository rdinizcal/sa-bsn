#include <gtest/gtest.h>

#include "FormulaRefs.hpp"

using namespace std;

class FormulaRefsTest : public testing::Test {
    protected:
        FormulaRefsTest() {}

        virtual void SetUp () {
        }
};

TEST_F(FormulaRefsTest, AddTaskRefs) {
    double reli = 0.9;
    double freq = 0.7;
    double cost = 0.5;

    FormulaRefs costFormulaRefs;

    costFormulaRefs.addTask("Task1", reli, freq, cost);
    
    /*
    costFormulaRefs.getReliabilityRef(leafTask1.getID());
    costFormulaRefs.getFrequencyRef(leafTask1.getID());
    costFormulaRefs.getCostRef(leafTask1.getID());
    
    std::vector<std::map<std::string, std::vector<double&>>> cost_formulae_refs = 
                                                                                {
                                                                                    leafTask1.getID(), 
                                                                                    {
                                                                                        cost_expr.getVariableReference(leafTask.getReliability().getID()),
                                                                                        cost_expr.getVariableReference(leafTask.getFrequency().getID(),
                                                                                        cost_expr.getVariableReference(leafTask.getCost().getID()))
                                                                                    }
                                                                                },
                                                                                {
                                                                                    leafTask2.getID(), 
                                                                                    {
                                                                                        cost_expr.getVariableReference(leafTask.getReliability().getID()),
                                                                                        cost_expr.getVariableReference(leafTask.getFrequency().getID(),
                                                                                        cost_expr.getVariableReference(leafTask.getCost().getID()))
                                                                                    }
                                                                                },
    */

    ASSERT_EQ(&costFormulaRefs.getReliabilityRef("Task1"), &reli);
    ASSERT_EQ(&costFormulaRefs.getFrequencyRef("Task1"), &freq);
    ASSERT_EQ(&costFormulaRefs.getCostRef("Task1"), &cost);
}