cmake_minimum_required(VERSION 2.8.3)
project(lepton)

add_compile_options(-std=c++11)

###########################################################################
# Set the search path for .cmake files.
SET (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake.Modules" ${CMAKE_MODULE_PATH})

# Add a local CMake module search path dependent on the desired installation destination.
# Thus, artifacts from the complete source build can be given precendence over any installed versions.
SET (CMAKE_MODULE_PATH "${CMAKE_INSTALL_PREFIX}/share/cmake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules" ${CMAKE_MODULE_PATH})

###########################################################################
# Include flags for compiling.
# INCLUDE (${CMAKE_CURRENT_SOURCE_DIR}/cmake.Modules/CompileFlags.cmake)

###########################################################################
# Find Catkin.
FIND_PACKAGE ( 
    catkin REQUIRED
)

###########################################################################
# Set linking libraries to successfully link test suites and binaries.
# SET (CMAKE_CXX_FLAGS "-Wall ${CMAKE_CXX_FLAGS}")

###########################################################################
# Set header files from OpenDaVINCI.
INCLUDE_DIRECTORIES (${catkin_INCLUDE_DIRS})
# Set include directory.
INCLUDE_DIRECTORIES(include)

###########################################################################
# Build project.
FILE(GLOB_RECURSE ${PROJECT_NAME}-sources "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
ADD_LIBRARY (${PROJECT_NAME}-core   OBJECT ${${PROJECT_NAME}-sources})
ADD_LIBRARY (${PROJECT_NAME}        STATIC $<TARGET_OBJECTS:${PROJECT_NAME}-core>)
TARGET_LINK_LIBRARIES (${PROJECT_NAME} ${catkin_LIBRARIES}) 

###########################################################################
# Export catkin package.
CATKIN_PACKAGE(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

###########################################################################
# Install library in local catkin environment.

INSTALL(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

INSTALL(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)