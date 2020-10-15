CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
PROJECT(test_suite)

add_compile_options(-std=c++11)

###########################################################################
## Find catkin and any catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg messages archlib)

###########################################################################
# Export catkin package.
CATKIN_PACKAGE()

###########################################################################
# Set catkin directory.
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})

# Set header files from BSN.
SET (BSN_INCLUDE_DIRS /usr/local/include/bsn)
INCLUDE_DIRECTORIES (${BSN_INCLUDE_DIRS})

# Set include directory.
INCLUDE_DIRECTORIES(include)

if(CATKIN_ENABLE_TESTING)
    # add gtest
    catkin_add_gtest(mytest src/mytest.cpp)

    # link required libraries
    target_link_libraries(mytest ${catkin_LIBRARIES} ${LIBRARIES})
endif()