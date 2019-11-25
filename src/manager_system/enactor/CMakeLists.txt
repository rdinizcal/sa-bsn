CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
PROJECT(enactor)

add_compile_options(-std=c++11)

###########################################################################
## Find catkin and any catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg roslib messages archlib)

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
# Set external libraries.
FIND_LIBRARY(BSN_LIBRARY bsn)
SET (LIBRARIES ${BSN_LIBRARY})

###########################################################################
# Build this project.
<<<<<<< HEAD:src/logging/logger/CMakeLists.txt
FILE(GLOB_RECURSE ${PROJECT_NAME}-srcs "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
ADD_LIBRARY (${PROJECT_NAME}lib ${${PROJECT_NAME}-srcs})
TARGET_LINK_LIBRARIES (${PROJECT_NAME}lib ${catkin_LIBRARIES} ${LIBRARIES}) 
ADD_DEPENDENCIES(${PROJECT_NAME}lib messages_generate_messages_cpp)

ADD_EXECUTABLE (${PROJECT_NAME} "${CMAKE_CURRENT_SOURCE_DIR}/apps/${PROJECT_NAME}.cpp" ${${PROJECT_NAME}-srcs})
TARGET_LINK_LIBRARIES (${PROJECT_NAME} PRIVATE cpprestsdk::cpprest ${catkin_LIBRARIES} ${LIBRARIES})
=======
FILE(GLOB ${PROJECT_NAME}-src "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

SET(enactor-src "${CMAKE_CURRENT_SOURCE_DIR}/src/Enactor.cpp")
ADD_EXECUTABLE (enactor  "${CMAKE_CURRENT_SOURCE_DIR}/apps/enactor.cpp" ${${PROJECT_NAME}-src} ${enactor-src})
TARGET_LINK_LIBRARIES (enactor ${catkin_LIBRARIES} ${LIBRARIES})
ADD_DEPENDENCIES(enactor messages_generate_messages_cpp)
>>>>>>> master:src/manager_system/enactor/CMakeLists.txt
