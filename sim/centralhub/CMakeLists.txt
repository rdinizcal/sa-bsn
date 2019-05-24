CMAKE_MINIMUM_REQUIRED (VERSION 3.5)
PROJECT(centralhub)

add_compile_options(-std=c++11)

## Find catkin and any catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs 
    genmsg
    bsn)

find_package(cpprestsdk REQUIRED NAMES cpprestsdk cpprest)

# Export catkin package.
CATKIN_PACKAGE(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS bsn message_runtime
)

###########################################################################
# Set catkin directory.
INCLUDE_DIRECTORIES(include ${catkin_INCLUDE_DIRS})

# Set include directory.
INCLUDE_DIRECTORIES(include)
SET (${LIBRARIES} lepton)

###########################################################################
# Build this project.
FILE(GLOB_RECURSE ${PROJECT_NAME}-srcs "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
ADD_EXECUTABLE (${PROJECT_NAME} "${CMAKE_CURRENT_SOURCE_DIR}/apps/centralhub.cpp" ${${PROJECT_NAME}-srcs} ${LIBRARIES})
# TARGET_LINK_LIBRARIES (${PROJECT_NAME} ${catkin_LIBRARIES}) 
target_link_libraries(${PROJECT_NAME} PRIVATE cpprestsdk::cpprest ${catkin_LIBRARIES})