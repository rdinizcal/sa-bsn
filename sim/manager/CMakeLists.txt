CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
PROJECT(manager)

add_compile_options(-std=c++11)

###########################################################################
## Find catkin and any catkin packages
FIND_PACKAGE(
    catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs 
    genmsg
    bsn
    lepton)

###########################################################################
# Export catkin package.
CATKIN_PACKAGE(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS bsn lepton message_runtime
)

###########################################################################
# Set catkin directory.
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})

# Set include directory.
INCLUDE_DIRECTORIES(include)

###########################################################################
# Build this project.
FILE(GLOB_RECURSE ${PROJECT_NAME}-srcs "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
## Necessary for tests
ADD_LIBRARY (${PROJECT_NAME}lib ${${PROJECT_NAME}-srcs})
TARGET_LINK_LIBRARIES (${PROJECT_NAME}lib ${catkin_LIBRARIES}) 

ADD_EXECUTABLE (${PROJECT_NAME} "${CMAKE_CURRENT_SOURCE_DIR}/apps/controller.cpp" ${${PROJECT_NAME}-srcs})
TARGET_LINK_LIBRARIES (${PROJECT_NAME} ${catkin_LIBRARIES})

###########################################################################
## Add gtest based cpp test target and link libraries
FILE(GLOB_RECURSE files "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp")

CATKIN_ADD_GTEST(${PROJECT_NAME}lib-test ${files})

IF(TARGET ${PROJECT_NAME}lib-test)
  TARGET_LINK_LIBRARIES(${PROJECT_NAME}lib-test ${PROJECT_NAME}lib)
ENDIF()

###########################################################################
