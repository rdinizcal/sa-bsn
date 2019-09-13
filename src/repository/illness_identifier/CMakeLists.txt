cmake_minimum_required(VERSION 2.8.3)
project(illness_identifier)

FIND_PACKAGE(catkin REQUIRED COMPONENTS message_generation messages rospy)

CATKIN_PACKAGE(
     CATKIN_DEPENDS rospy messages
)

INCLUDE_DIRECTORIES(
  ${catkin_INCLUDE_DIRS}
)