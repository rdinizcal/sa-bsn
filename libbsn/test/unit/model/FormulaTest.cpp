#include <gtest/gtest.h>

#include "model/Formula.hpp"

class FormulaTest : public testing::Test {
    protected:
        FormulaTest() {}

        virtual void SetUp() {
        }
};

TEST_F(FormulaTest, SimpleConstruct) {

    try {
        bsn::model::Formula formula("x+y");
    } catch (const std::exception& e){
        FAIL() << e.what();
    }

    ASSERT_TRUE(true);
}

TEST_F(FormulaTest, ApplyForOneParameterFormula) {
    bsn::model::Formula formula("x+x");
    double answer = 0;

    try {
        answer = formula.apply({"x"}, {2});

    } catch (const std::exception& e){
        FAIL() << e.what();
    }

    ASSERT_EQ(answer, 4);
}

TEST_F(FormulaTest, ApplyNonExistentParameter) {
    bsn::model::Formula formula("x+x");
    double answer = 0;

    try {
        answer = formula.apply({"y"}, {2});
        ASSERT_EQ(answer, 4);
    } catch (const std::exception&){
        ASSERT_TRUE(true);
    }
}

TEST_F(FormulaTest, ApplyForTwoParameterFormula) {
    bsn::model::Formula formula("x+y");
    std::vector<std::string> parameters = {"x","y"};
    std::vector<double> values = {2,7};
    double answer = 0;

    try {
        answer = formula.apply(parameters, values);
    } catch (const std::exception& e){
        FAIL() << e.what();
    }

    ASSERT_EQ(answer, 9);
}

TEST_F(FormulaTest, ApplyForDistinctNumberOfParameters) {
    bsn::model::Formula formula("x+y");
    std::vector<std::string> parameters = {"x","y"};
    std::vector<double> values = {2};
    double answer = 0;

    try {
        answer = formula.apply(parameters, values);
        ASSERT_EQ(answer, 4);
    } catch (const std::length_error& e){
        ASSERT_TRUE(true);   
    }
}