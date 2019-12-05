cmake_minimum_required(VERSION 2.8.3)
project(analyzer)

FIND_PACKAGE(catkin REQUIRED COMPONENTS message_generation messages rospy archlib)

CATKIN_PACKAGE(
    CATKIN_DEPENDS rospy messages archlib
)

INCLUDE_DIRECTORIES(
  ${catkin_INCLUDE_DIRS}
)