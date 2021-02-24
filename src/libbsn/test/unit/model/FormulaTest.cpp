#include <gtest/gtest.h>

#include "libbsn/model/Formula.hpp"

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

TEST_F(FormulaTest, ConstructWithTermsAndValues) {

    try {
        bsn::model::Formula formula("x+y",{"x","y"},{1,2});
    } catch (const std::exception& e){
        FAIL() << e.what();
    }

    ASSERT_TRUE(true);
}

TEST_F(FormulaTest, InvalidFormulaConstruction) {

    try {
        bsn::model::Formula formula("x+y",{"x","y","z"},{1,2});
        FAIL();
    } catch (const std::length_error& e){
        ASSERT_TRUE(true);
    }
}


TEST_F(FormulaTest, EvaluateForOneParameterFormula) {
    bsn::model::Formula formula("x+x",{"x"}, {2});
    double answer = 0;

    try {
        answer = formula.evaluate();

    } catch (const std::exception& e){
        FAIL() << e.what();
    }

    ASSERT_EQ(answer, 4);
}

TEST_F(FormulaTest, EvaluateNonExistentParameter) {
    bsn::model::Formula formula("x+x", {"y"}, {2});
    double answer = 0;

    try {
        answer = formula.evaluate();
        FAIL();
    } catch (const std::exception&){
        ASSERT_TRUE(true);
    }
}

TEST_F(FormulaTest, EvaluateForTwoParameterFormula) {
    std::vector<std::string> terms = {"x","y"};
    std::vector<double> values = {2,7};
    bsn::model::Formula formula("x+y", terms, values);
    double answer = 0;

    try {
        answer = formula.evaluate();
    } catch (const std::exception& e){
        FAIL() << e.what();
    }

    ASSERT_EQ(answer, 9);
}