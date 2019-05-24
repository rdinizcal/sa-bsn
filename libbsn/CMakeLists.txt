cmake_minimum_required(VERSION 2.8.3)
project(bsn)

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
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

###########################################################################
# Set message generation.
ADD_MESSAGE_FILES(
  FILES
  info/ContextInfo.msg
  info/TaskInfo.msg
  info/SystemInfo.msg
  data/SensorData.msg
  command/ControlCommand.msg
  command/ControlCentralhub.msg
)

GENERATE_MESSAGES(
  DEPENDENCIES
  std_msgs
)

###########################################################################
# Export catkin package.
CATKIN_PACKAGE(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime
)

###########################################################################
# Set header files from Catkin.
INCLUDE_DIRECTORIES (${catkin_INCLUDE_DIRS})
# Set include directory.
INCLUDE_DIRECTORIES(include)

###########################################################################
# Set external libraries.
LINK_DIRECTORIES(/usr/local/lib)
SET(LIBRARIES lepton)

###########################################################################
# Build project.
FILE(GLOB_RECURSE ${PROJECT_NAME}-srcs "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
ADD_LIBRARY(${PROJECT_NAME} ${${PROJECT_NAME}-srcs})
TARGET_LINK_LIBRARIES (${PROJECT_NAME} ${catkin_LIBRARIES} ${LIBRARIES}) 

###########################################################################
## Add gtest based cpp test target and link libraries
FILE(GLOB_RECURSE files "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp")

CATKIN_ADD_GTEST(${PROJECT_NAME}-test ${files})

IF(TARGET ${PROJECT_NAME}-test)
  TARGET_LINK_LIBRARIES(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES} ${LIBRARIES})
ENDIF()

###########################################################################
# Install library in local catkin environment.

INSTALL(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

INSTALL(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)