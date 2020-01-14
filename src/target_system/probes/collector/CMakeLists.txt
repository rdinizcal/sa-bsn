CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
PROJECT(probe)

add_compile_options(-std=c++11)

###########################################################################
## Find catkin and any catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg messages archlib)

###########################################################################
# Export catkin package.
CATKIN_PACKAGE(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS messages message_runtime archlib
)

###########################################################################
# Set catkin directory.
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})

# Set include directory.
#INCLUDE_DIRECTORIES(include)

###########################################################################
# Build this project.
ADD_EXECUTABLE (collector "${CMAKE_CURRENT_SOURCE_DIR}/apps/collector.cpp")
TARGET_LINK_LIBRARIES (collector ${catkin_LIBRARIES} ${LIBRARIES})
ADD_DEPENDENCIES(collector messages_generate_messages_cpp)