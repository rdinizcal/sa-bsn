/* Generated file, do not edit */

#ifndef CXXTEST_RUNNING
#define CXXTEST_RUNNING
#endif

#define _CXXTEST_HAVE_STD
#define _CXXTEST_HAVE_EH
#include <cxxtest/TestListener.h>
#include <cxxtest/TestTracker.h>
#include <cxxtest/TestRunner.h>
#include <cxxtest/RealDescriptions.h>
#include <cxxtest/TestMain.h>
#include <cxxtest/ErrorPrinter.h>

int main( int argc, char *argv[] ) {
 int status;
    CxxTest::ErrorPrinter tmp;
    CxxTest::RealWorldDescription::_worldName = "cxxtest";
    status = CxxTest::Main< CxxTest::ErrorPrinter >( tmp, argc, argv );
    return status;
}
bool suite_SensoConfigurationTestSuite_init = false;
#include "/home/rdiniz/projects/seams19/bsn/libbsn/test/test_configuration.h"

static SensoConfigurationTestSuite suite_SensoConfigurationTestSuite;

static CxxTest::List Tests_SensoConfigurationTestSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_SensoConfigurationTestSuite( "libbsn/test/test_configuration.h", 10, "SensoConfigurationTestSuite", suite_SensoConfigurationTestSuite, Tests_SensoConfigurationTestSuite );

static class TestDescription_suite_SensoConfigurationTestSuite_test_constructor : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_SensoConfigurationTestSuite_test_constructor() : CxxTest::RealTestDescription( Tests_SensoConfigurationTestSuite, suiteDescription_SensoConfigurationTestSuite, 46, "test_constructor" ) {}
 void runTest() { suite_SensoConfigurationTestSuite.test_constructor(); }
} testDescription_suite_SensoConfigurationTestSuite_test_constructor;

static class TestDescription_suite_SensoConfigurationTestSuite_test_unknow_value : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_SensoConfigurationTestSuite_test_unknow_value() : CxxTest::RealTestDescription( Tests_SensoConfigurationTestSuite, suiteDescription_SensoConfigurationTestSuite, 55, "test_unknow_value" ) {}
 void runTest() { suite_SensoConfigurationTestSuite.test_unknow_value(); }
} testDescription_suite_SensoConfigurationTestSuite_test_unknow_value;

static class TestDescription_suite_SensoConfigurationTestSuite_test_getDisplacement_normal : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_SensoConfigurationTestSuite_test_getDisplacement_normal() : CxxTest::RealTestDescription( Tests_SensoConfigurationTestSuite, suiteDescription_SensoConfigurationTestSuite, 61, "test_getDisplacement_normal" ) {}
 void runTest() { suite_SensoConfigurationTestSuite.test_getDisplacement_normal(); }
} testDescription_suite_SensoConfigurationTestSuite_test_getDisplacement_normal;

static class TestDescription_suite_SensoConfigurationTestSuite_test_getDisplacement_inverse : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_SensoConfigurationTestSuite_test_getDisplacement_inverse() : CxxTest::RealTestDescription( Tests_SensoConfigurationTestSuite, suiteDescription_SensoConfigurationTestSuite, 73, "test_getDisplacement_inverse" ) {}
 void runTest() { suite_SensoConfigurationTestSuite.test_getDisplacement_inverse(); }
} testDescription_suite_SensoConfigurationTestSuite_test_getDisplacement_inverse;

static class TestDescription_suite_SensoConfigurationTestSuite_test_getDisplacement_medium : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_SensoConfigurationTestSuite_test_getDisplacement_medium() : CxxTest::RealTestDescription( Tests_SensoConfigurationTestSuite, suiteDescription_SensoConfigurationTestSuite, 85, "test_getDisplacement_medium" ) {}
 void runTest() { suite_SensoConfigurationTestSuite.test_getDisplacement_medium(); }
} testDescription_suite_SensoConfigurationTestSuite_test_getDisplacement_medium;

static class TestDescription_suite_SensoConfigurationTestSuite_test_getDisplacement_invalid_argument : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_SensoConfigurationTestSuite_test_getDisplacement_invalid_argument() : CxxTest::RealTestDescription( Tests_SensoConfigurationTestSuite, suiteDescription_SensoConfigurationTestSuite, 101, "test_getDisplacement_invalid_argument" ) {}
 void runTest() { suite_SensoConfigurationTestSuite.test_getDisplacement_invalid_argument(); }
} testDescription_suite_SensoConfigurationTestSuite_test_getDisplacement_invalid_argument;

static class TestDescription_suite_SensoConfigurationTestSuite_test_low : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_SensoConfigurationTestSuite_test_low() : CxxTest::RealTestDescription( Tests_SensoConfigurationTestSuite, suiteDescription_SensoConfigurationTestSuite, 109, "test_low" ) {}
 void runTest() { suite_SensoConfigurationTestSuite.test_low(); }
} testDescription_suite_SensoConfigurationTestSuite_test_low;

static class TestDescription_suite_SensoConfigurationTestSuite_test_medium0 : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_SensoConfigurationTestSuite_test_medium0() : CxxTest::RealTestDescription( Tests_SensoConfigurationTestSuite, suiteDescription_SensoConfigurationTestSuite, 122, "test_medium0" ) {}
 void runTest() { suite_SensoConfigurationTestSuite.test_medium0(); }
} testDescription_suite_SensoConfigurationTestSuite_test_medium0;

static class TestDescription_suite_SensoConfigurationTestSuite_test_medium1 : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_SensoConfigurationTestSuite_test_medium1() : CxxTest::RealTestDescription( Tests_SensoConfigurationTestSuite, suiteDescription_SensoConfigurationTestSuite, 131, "test_medium1" ) {}
 void runTest() { suite_SensoConfigurationTestSuite.test_medium1(); }
} testDescription_suite_SensoConfigurationTestSuite_test_medium1;

static class TestDescription_suite_SensoConfigurationTestSuite_test_high0 : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_SensoConfigurationTestSuite_test_high0() : CxxTest::RealTestDescription( Tests_SensoConfigurationTestSuite, suiteDescription_SensoConfigurationTestSuite, 141, "test_high0" ) {}
 void runTest() { suite_SensoConfigurationTestSuite.test_high0(); }
} testDescription_suite_SensoConfigurationTestSuite_test_high0;

static class TestDescription_suite_SensoConfigurationTestSuite_test_high1 : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_SensoConfigurationTestSuite_test_high1() : CxxTest::RealTestDescription( Tests_SensoConfigurationTestSuite, suiteDescription_SensoConfigurationTestSuite, 151, "test_high1" ) {}
 void runTest() { suite_SensoConfigurationTestSuite.test_high1(); }
} testDescription_suite_SensoConfigurationTestSuite_test_high1;

#include <cxxtest/Root.cpp>
const char* CxxTest::RealWorldDescription::_worldName = "cxxtest";
