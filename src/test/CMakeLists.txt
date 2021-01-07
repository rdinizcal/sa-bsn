CMAKE_MINIMUM_REQUIRED (VERSION 3.0.2)
PROJECT(test_suite)

add_compile_options(-std=c++11)

###########################################################################
## Find catkin and any catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg messages archlib component roslib)

###########################################################################
# Export catkin package.
CATKIN_PACKAGE(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS messages message_runtime archlib
)

###########################################################################
# Set catkin directory.
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
message( ${CMAKE_CURRENT_SOURCE_DIR} )

# Set include directory.
INCLUDE_DIRECTORIES(include)

#LINK_DIRECTORIES(include)

###########################################################################
#FILE(GLOB_RECURSE ${PROJECT_NAME}-srcs "${PROJECT_SOURCE_DIR}/src/*.cpp")
#ADD_LIBRARY(${PROJECT_NAME} ${${PROJECT_NAME}-srcs})
#ADD_EXECUTABLE (sensor_test src/SensorTest.cpp src/CentralhubTest.cpp)
#TARGET_LINK_LIBRARIES (sensor_test ${catkin_LIBRARIES} include)
#ADD_DEPENDENCIES(sensor_test)


###########################################################################
# Build this project.
#FILE(GLOB ${PROJECT_NAME}-src "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

SET(sensor_test-src "${CMAKE_CURRENT_SOURCE_DIR}/src/SensorTestClass.cpp")
SET(centralhub_test-src "${CMAKE_CURRENT_SOURCE_DIR}/src/CentralhubTestClass.cpp")
#ADD_LIBRARY (test_node "${CMAKE_CURRENT_SOURCE_DIR}/apps/testnode.cpp" 
#            ${${PROJECT_NAME}-src} 
#            ${sensor_test-src} 
#            ${centralhub_test-src})
ADD_LIBRARY (test_node 
            ${${PROJECT_NAME}-src} 
            ${sensor_test-src} 
            ${centralhub_test-src})
TARGET_LINK_LIBRARIES (test_node ${catkin_LIBRARIES} ${LIBRARIES})
ADD_DEPENDENCIES(test_node messages_generate_messages_cpp)


###########################################################################
## Add gtest based cpp test target and link libraries
ENABLE_TESTING()

if(CATKIN_ENABLE_TESTING)

    find_package(rostest REQUIRED)
    # add gtest
    add_rostest_gtest(
        testnode 
        testnode.launch 
        apps/main.cpp
        src/G4T1Test.cpp
        src/G3T11Test.cpp
        src/G3T12Test.cpp
        src/G3T13Test.cpp
        src/G3T14Test.cpp
        src/G3T15Test.cpp
        )

    # link required libraries
    target_link_libraries(
        testnode
        ${catkin_LIBRARIES} 
        ${LIBRARIES}
        test_node
        )
endif()
