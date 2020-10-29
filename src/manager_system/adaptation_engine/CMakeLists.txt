CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
PROJECT(adaptation_engine)

add_compile_options(-std=c++11)

###########################################################################
## Find catkin and any catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg roslib messages archlib)
FIND_PACKAGE(cpprestsdk REQUIRED NAMES cpprestsdk cpprest)

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

# Set header files from BSN.
SET (LEPTON_INCLUDE_DIRS /usr/local/include/lepton)
INCLUDE_DIRECTORIES (${LEPTON_INCLUDE_DIRS})

# Set include directory.
INCLUDE_DIRECTORIES(include)

###########################################################################
# Set external libraries.
FIND_LIBRARY(BSN_LIBRARY bsn)
FIND_LIBRARY(LEPTON_LIBRARY lepton)
SET (LIBRARIES ${BSN_LIBRARY} ${LEPTON_LIBRARY})

###########################################################################
# Build this project.
FILE(GLOB ${PROJECT_NAME}-src "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

SET(engine-src "${CMAKE_CURRENT_SOURCE_DIR}/src/Engine.cpp")
ADD_EXECUTABLE (engine  "${CMAKE_CURRENT_SOURCE_DIR}/apps/engine.cpp" ${${PROJECT_NAME}-src} ${engine-src})
TARGET_LINK_LIBRARIES (engine ${catkin_LIBRARIES} ${LIBRARIES})
ADD_DEPENDENCIES(engine messages_generate_messages_cpp)

###########################################################################
# Include tests

#if(CATKIN_ENABLE_TESTING)
    # add gtest
#    catkin_add_gtest(${PROJECT_NAME}-test test/adaptation_engine-test.cpp)

 #   if (TARGET ${PROJECT_NAME}-test)
 #       # link required libraries
 #       target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
 #   endif()
#endif()

###########################################################################
# Install this project.
#INSTALL(TARGETS ${PROJECT_NAME}
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#INSTALL(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})