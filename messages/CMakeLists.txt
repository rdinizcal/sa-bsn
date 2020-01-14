cmake_minimum_required(VERSION 2.8.3)
project(messages)

add_compile_options(-std=c++11)

###########################################################################
# Set the search path for .cmake files.
SET (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake.Modules" ${CMAKE_MODULE_PATH})

# Add a local CMake module search path dependent on the desired installation destination.
# Thus, artifacts from the complete source build can be given precendence over any installed versions.
SET (CMAKE_MODULE_PATH "${CMAKE_INSTALL_PREFIX}/share/cmake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules" ${CMAKE_MODULE_PATH})

###########################################################################
# Include flags for compiling.
INCLUDE (${CMAKE_CURRENT_SOURCE_DIR}/cmake.Modules/CompileFlags.cmake)
# Set compilation flags
SET (CMAKE_CXX_FLAGS "-Wall ${CMAKE_CXX_FLAGS}")

###########################################################################
# Find Catkin.
FIND_PACKAGE(catkin REQUIRED COMPONENTS std_msgs message_generation)

###########################################################################
# Set message generation.
ADD_MESSAGE_FILES( FILES
  target_system/internal/SensorData.msg
  target_system/external/TargetSystemData.msg
  system_manager/external/ReconfigurationCommand.msg
  system_manager/internal/CommandControl.msg
)

GENERATE_MESSAGES(DEPENDENCIES std_msgs)

###########################################################################
# Export catkin package.
CATKIN_PACKAGE(CATKIN_DEPENDS message_runtime std_msgs)