CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
PROJECT(g3t1_1)

add_compile_options(-std=c++11)

## Find catkin and any catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg messages)

# Export catkin package.
CATKIN_PACKAGE(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS messages message_runtime
)

###########################################################################
# Set catkin directory.
INCLUDE_DIRECTORIES(include ${catkin_INCLUDE_DIRS})

# Set header files from BSN.
SET (BSN_INCLUDE_DIRS /usr/local/include/bsn)
INCLUDE_DIRECTORIES (${BSN_INCLUDE_DIRS})

# Set include directory.
INCLUDE_DIRECTORIES(include)

###########################################################################
# Add and Set linking libraries to successfully link test suites and binaries.
FIND_LIBRARY(BSN_LIBRARY bsn)
SET (LIBRARIES ${BSN_LIBRARY})

###########################################################################
# Build this project.
FILE(GLOB_RECURSE ${PROJECT_NAME}-srcs "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
ADD_EXECUTABLE (${PROJECT_NAME} "${CMAKE_CURRENT_SOURCE_DIR}/apps/${PROJECT_NAME}.cpp" ${${PROJECT_NAME}-srcs})
TARGET_LINK_LIBRARIES (${PROJECT_NAME} ${catkin_LIBRARIES} ${LIBRARIES}) 
ADD_DEPENDENCIES(${PROJECT_NAME} messages_generate_messages_cpp)
