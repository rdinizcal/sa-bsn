cmake_minimum_required(VERSION 3.0.2)
project(test)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

###########################################################################
## Add gtest based cpp test target and link libraries
ENABLE_TESTING()
FIND_PACKAGE(GTest REQUIRED)
INCLUDE_DIRECTORIES(${GTEST_INCLUDE_DIRS})

FILE(GLOB_RECURSE files "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp")
ADD_EXECUTABLE(run_test ${files})
TARGET_LINK_LIBRARIES(run_test ${PROJECT_NAME} ${LIBRARIES} ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)
ADD_TEST(run_test run_test)

##if(CATKIN_ENABLE_TESTING)
##  find_package(rostest REQUIRED)
##  find_package(gtest)
##  add_rostest_gtest(tests_mynode test/alaunch.launch test/main.cpp)
##  target_link_libraries(tests_mynode ${catkin_LIBRARIES})
##endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
)