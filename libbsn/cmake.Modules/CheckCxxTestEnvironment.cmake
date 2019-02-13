FIND_PACKAGE (PythonInterp 2.7 REQUIRED)

IF(PYTHONINTERP_FOUND)
    SET (CXXTEST_USE_PYTHON true)
    SET (CXXTEST_TESTGEN_ARGS --xunit-printer --have-eh)
    SET (CXXTEST_PYTHON_INTERPRETER "${PYTHON_EXECUTABLE}")
ENDIF()

###########################################################################
# Next, find CxxTest (shipped with the distribution).
FIND_PACKAGE(CxxTest)
IF(CXXTEST_FOUND)
    INCLUDE_DIRECTORIES(${CXXTEST_INCLUDE_DIRS})
    ENABLE_TESTING()
    MESSAGE(STATUS "Found CxxTest: Compiling and running test suites enabled.")
ENDIF(CXXTEST_FOUND)