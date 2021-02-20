CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
PROJECT(injector)

add_compile_options(-std=c++11)

###########################################################################
## Find catkin and any catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg roslib messages archlib libbsn)

###########################################################################
# Export catkin package.
CATKIN_PACKAGE(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS messages message_runtime archlib libbsn
)

###########################################################################
# Set catkin directory.
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})

# Set include directory.
INCLUDE_DIRECTORIES(include)

###########################################################################
# Build this project.
FILE(GLOB ${PROJECT_NAME}-src "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

SET(injector-src "${CMAKE_CURRENT_SOURCE_DIR}/src/Injector.cpp")
ADD_EXECUTABLE (injector  "${CMAKE_CURRENT_SOURCE_DIR}/apps/injector.cpp" ${${PROJECT_NAME}-src} ${injector-src})
TARGET_LINK_LIBRARIES (injector ${catkin_LIBRARIES} ${LIBRARIES})
ADD_DEPENDENCIES(injector messages_generate_messages_cpp)