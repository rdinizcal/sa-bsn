cmake_minimum_required(VERSION 3.0.2)
project(test)

catkin_package()

find_package(catkin REQUIRED COMPONENTS roscpp)

###########################################################################
## Add gtest based cpp test target and link libraries
ENABLE_TESTING()
#INCLUDE_DIRECTORIES(${GTEST_INCLUDE_DIRS})

#FILE(GLOB_RECURSE files "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
#TARGET_LINK_LIBRARIES(run_test ${PROJECT_NAME} ${LIBRARIES} ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)
#ADD_TEST(run_test run_test)
#  add_executable(run_test ${files})
#  add_executable(test_run ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(GTest REQUIRED)
  catkin_add_gtest(utest ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
#  add_rostest_gtest(test_mynode ${CMAKE_CURRENT_SOURCE_DIR}/alaunch.launch ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
#  target_link_libraries(test_mynode ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
#  target_link_libraries(test_run ${PROJECT_NAME} ${LIBRARIES} ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
)