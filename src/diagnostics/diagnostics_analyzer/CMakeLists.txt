CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
PROJECT(diagnostics_analyzer)

add_compile_options(-std=c++11)

###########################################################################
## Find catkin and any catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp roslib std_msgs genmsg messages archlib)

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

# Set header files from BSN.
SET (BSN_INCLUDE_DIRS /usr/local/include/bsn)
INCLUDE_DIRECTORIES (${BSN_INCLUDE_DIRS})

# Set include directory.
INCLUDE_DIRECTORIES(include)

###########################################################################
# Add and Set linking libraries to successfully link binaries.
FIND_LIBRARY(BSN_LIBRARY bsn)
SET (LIBRARIES ${BSN_LIBRARY})

###########################################################################
# Build this project.
FILE(GLOB ${PROJECT_NAME}-src "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

SET(diangostics_analyzer-src "${CMAKE_CURRENT_SOURCE_DIR}/src/PropertyAnalyzer.cpp")
ADD_EXECUTABLE (diagnostics_analyzer  "${CMAKE_CURRENT_SOURCE_DIR}/apps/diagnostics.cpp" ${${PROJECT_NAME}-src} ${diagnostics_analyzer-src})
TARGET_LINK_LIBRARIES (diagnostics_analyzer ${catkin_LIBRARIES} ${LIBRARIES})
ADD_DEPENDENCIES(diagnostics_analyzer services_generate_messages_cpp)
