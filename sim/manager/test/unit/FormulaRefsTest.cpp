#include <gtest/gtest.h>

//#include "FormulaRefs.hpp"

using namespace std;
/* 
class FormulaRefsTest : public testing::Test {
    protected:
        FormulaRefsTest() {}

        virtual void SetUp () {
        }
};

TEST_F(FormulaRefsTest, CostFormulaAddTaskRefs) {
    double reli = 0.9;
    double freq = 0.7;
    double cost = 0.5;

    FormulaRefs costFormulaRefs;

    costFormulaRefs.addTask("Task1", reli, freq, cost);

    ASSERT_EQ(&costFormulaRefs.getReliabilityRef("Task1"), &reli);
    ASSERT_EQ(&costFormulaRefs.getFrequencyRef("Task1"), &freq);
    ASSERT_EQ(&costFormulaRefs.getCostRef("Task1"), &cost);
}

TEST_F(FormulaRefsTest, ReliabilityFormulaAddTaskRefs) {
    double reli = 0.9;
    double freq = 0.7;

    FormulaRefs reliFormulaRefs;

    reliFormulaRefs.addTask("Task1", reli, freq);

    ASSERT_EQ(&reliFormulaRefs.getReliabilityRef("Task1"), &reli);
    ASSERT_EQ(&reliFormulaRefs.getFrequencyRef("Task1"), &freq);
}

TEST_F(FormulaRefsTest, AddThreeTaskRefs) {
    double reli1 = 0.9;
    double freq1 = 0.7;
    double cost1 = 0.5;

    double reli2 = 0.5;
    double freq2 = 0.1;
    double cost2 = 0.9;

    double reli3 = 1;
    double freq3 = 0.1;
    double cost3 = 0.12;

    FormulaRefs costFormulaRefs;

    costFormulaRefs.addTask("Task1", reli1, freq1, cost1);
    costFormulaRefs.addTask("Task2", reli2, freq2, cost2);
    costFormulaRefs.addTask("Task3", reli3, freq3, cost3);


    ASSERT_EQ(&costFormulaRefs.getReliabilityRef("Task1"), &reli1);
    ASSERT_EQ(&costFormulaRefs.getFrequencyRef("Task1"), &freq1);
    ASSERT_EQ(&costFormulaRefs.getCostRef("Task1"), &cost1);

    ASSERT_EQ(&costFormulaRefs.getReliabilityRef("Task2"), &reli2);
    ASSERT_EQ(&costFormulaRefs.getFrequencyRef("Task2"), &freq2);
    ASSERT_EQ(&costFormulaRefs.getCostRef("Task2"), &cost2);

    ASSERT_EQ(&costFormulaRefs.getReliabilityRef("Task3"), &reli3);
    ASSERT_EQ(&costFormulaRefs.getFrequencyRef("Task3"), &freq3);
    ASSERT_EQ(&costFormulaRefs.getCostRef("Task3"), &cost3);
} */