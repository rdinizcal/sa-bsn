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
# Set include directory.
INCLUDE_DIRECTORIES(include)
  
###########################################################################
# Add and Set linking libraries to successfully link test suites and binaries.
FIND_LIBRARY(LEPTON_LIBRARY lepton)
SET (LIBRARIES ${LEPTON_LIBRARY})

###########################################################################
# Build project.
FILE(GLOB_RECURSE ${PROJECT_NAME}-srcs "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
ADD_LIBRARY(${PROJECT_NAME} ${${PROJECT_NAME}-srcs})
TARGET_LINK_LIBRARIES (${PROJECT_NAME} ${LIBRARIES}) 

###########################################################################
## Add gtest based cpp test target and link libraries
ENABLE_TESTING()
FIND_PACKAGE(GTest REQUIRED)
INCLUDE_DIRECTORIES(${GTEST_INCLUDE_DIRS})

FILE(GLOB_RECURSE files "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp")
ADD_EXECUTABLE(run_test ${files})
TARGET_LINK_LIBRARIES(run_test ${PROJECT_NAME} ${LIBRARIES} ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)
ADD_TEST(run_test run_test)

###########################################################################
# Install library
INSTALL(TARGETS ${PROJECT_NAME} DESTINATION lib COMPONENT lib)
INSTALL(DIRECTORY include/ DESTINATION include/bsn COMPONENT lib)
