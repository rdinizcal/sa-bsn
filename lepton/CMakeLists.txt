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
INCLUDE (${CMAKE_CURRENT_SOURCE_DIR}/cmake.Modules/CompileFlags.cmake)
# Set compilation flags
SET (CMAKE_CXX_FLAGS "-Wall ${CMAKE_CXX_FLAGS}")

###########################################################################
# Set include directory.
INCLUDE_DIRECTORIES(include)

###########################################################################
# Build project.
FILE(GLOB_RECURSE ${PROJECT_NAME}-srcs "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
ADD_LIBRARY(${PROJECT_NAME} ${${PROJECT_NAME}-srcs})
TARGET_LINK_LIBRARIES (${PROJECT_NAME}) 

###########################################################################
# Install library
INSTALL(TARGETS ${PROJECT_NAME} DESTINATION lib COMPONENT lib)
INSTALL(DIRECTORY include/ DESTINATION include/lepton COMPONENT lib)
