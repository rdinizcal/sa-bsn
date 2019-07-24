CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
PROJECT(logger)

add_compile_options(-std=c++11)

###########################################################################
## Find catkin and any catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg roslib messages)
FIND_PACKAGE(cpprestsdk REQUIRED NAMES cpprestsdk cpprest)

###########################################################################
# Export catkin package.
CATKIN_PACKAGE(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS messages message_runtime
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
# Set external libraries.
FIND_LIBRARY(LEPTON_LIBRARY lepton)
FIND_LIBRARY(BSN_LIBRARY bsn)
SET (LIBRARIES ${BSN_LIBRARY} ${LEPTON_LIBRARY})

###########################################################################
# Build this project.
FILE(GLOB_RECURSE ${PROJECT_NAME}-srcs "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
ADD_LIBRARY (${PROJECT_NAME}lib ${${PROJECT_NAME}-srcs})
TARGET_LINK_LIBRARIES (${PROJECT_NAME}lib ${catkin_LIBRARIES} ${LIBRARIES}) 

ADD_EXECUTABLE (${PROJECT_NAME} "${CMAKE_CURRENT_SOURCE_DIR}/apps/${PROJECT_NAME}.cpp" ${${PROJECT_NAME}-srcs})
TARGET_LINK_LIBRARIES (${PROJECT_NAME} PRIVATE cpprestsdk::cpprest ${catkin_LIBRARIES} ${LIBRARIES})
ADD_DEPENDENCIES(${PROJECT_NAME} messages_generate_messages_cpp)