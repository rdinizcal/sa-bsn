CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
PROJECT(adaptation_engine)

add_compile_options(-std=c++11)

###########################################################################
## Find catkin and any catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg roslib messages archlib lepton libbsn)

###########################################################################
# Export catkin package.
CATKIN_PACKAGE(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS messages message_runtime archlib lepton libbsn
)

###########################################################################
# Set catkin directory.
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})

# Set include directory.
INCLUDE_DIRECTORIES(include)

###########################################################################
# Build this project.
FILE(GLOB ${PROJECT_NAME}-src "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

ADD_EXECUTABLE (reli_engine  "${CMAKE_CURRENT_SOURCE_DIR}/apps/reli_engine.cpp" ${${PROJECT_NAME}-src} )
TARGET_LINK_LIBRARIES (reli_engine ${catkin_LIBRARIES} ${LIBRARIES})
ADD_DEPENDENCIES(reli_engine messages_generate_messages_cpp)

ADD_EXECUTABLE (cost_engine  "${CMAKE_CURRENT_SOURCE_DIR}/apps/cost_engine.cpp" ${${PROJECT_NAME}-src} )
TARGET_LINK_LIBRARIES (cost_engine ${catkin_LIBRARIES} ${LIBRARIES})
ADD_DEPENDENCIES(cost_engine messages_generate_messages_cpp)

###########################################################################
# Install this project.
#INSTALL(TARGETS ${PROJECT_NAME}
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#INSTALL(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})