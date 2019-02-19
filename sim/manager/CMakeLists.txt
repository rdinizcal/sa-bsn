CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)

PROJECT(manager)

## Find catkin and any catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS 
    roscpp 
    rospy 
    std_msgs 
    genmsg
    bsn)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#GENERATE_MESSAGES(DEPENDENCIES std_msgs)

## Declare a catkin package
CATKIN_PACKAGE()

###########################################################################
# Set catkin directory.
INCLUDE_DIRECTORIES(include ${catkin_INCLUDE_DIRS})

# Set include directory.
INCLUDE_DIRECTORIES(include)

###########################################################################
# Build this project.
FILE(GLOB_RECURSE thisproject-sources "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
ADD_LIBRARY (${PROJECT_NAME}lib-static STATIC ${thisproject-sources})
ADD_EXECUTABLE (controller "${CMAKE_CURRENT_SOURCE_DIR}/apps/controller.cpp")
TARGET_LINK_LIBRARIES (controller ${PROJECT_NAME}lib-static ${catkin_LIBRARIES}) 
ADD_DEPENDENCIES(controller manager_generate_messages_cpp)
