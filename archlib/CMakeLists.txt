CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
PROJECT(archlib)

add_compile_options(-std=c++11 -g)

## Find catkin and any catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg message_generation)

###########################################################################
# Set message generation.
ADD_MESSAGE_FILES( FILES
  target_system/external/Status.msg
  target_system/external/EnergyStatus.msg
  target_system/external/Event.msg
  system_manager/external/AdaptationCommand.msg
  system_manager/internal/Strategy.msg
  system_manager/internal/Exception.msg
  knowledge_repository/external/Persist.msg
  simulation/external/Uncertainty.msg
)

ADD_SERVICE_FILES( FILES
  knowledge_repository/external/DataAccessRequest.srv
  target_system/internal/EffectorRegister.srv
  system_manager/internal/EngineRequest.srv
)

GENERATE_MESSAGES(DEPENDENCIES std_msgs)

###########################################################################
# Export catkin package.
CATKIN_PACKAGE(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS std_msgs message_runtime
)

###########################################################################
# Set catkin directory.
INCLUDE_DIRECTORIES(include ${catkin_INCLUDE_DIRS})

# Set include directory.
INCLUDE_DIRECTORIES(include)

###########################################################################
# Build this project.
FILE(GLOB_RECURSE ${PROJECT_NAME}-src "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
ADD_LIBRARY(${PROJECT_NAME} ${${PROJECT_NAME}-src})
TARGET_LINK_LIBRARIES (${PROJECT_NAME} ${catkin_LIBRARIES} ${LIBRARIES}) 
ADD_DEPENDENCIES(${PROJECT_NAME} messages_generate_messages_cpp services_generate_messages_cpp)

###########################################################################
# Install this project.
INSTALL(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

INSTALL(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
