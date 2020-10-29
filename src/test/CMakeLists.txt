CMAKE_MINIMUM_REQUIRED (VERSION 3.0.2)
PROJECT(test_suite)

add_compile_options(-std=c++11)

###########################################################################
## Find catkin and any catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg messages archlib component roslib)

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


###########################################################################
## Add gtest based cpp test target and link libraries
ENABLE_TESTING()

if(CATKIN_ENABLE_TESTING)

    find_package(rostest REQUIRED)
    # add gtest
    add_rostest_gtest(
        testnode 
        testnode.launch 
        src/testnode.cpp
        )

    # link required libraries
    target_link_libraries(
        testnode 
        ${catkin_LIBRARIES} 
        ${LIBRARIES}
        )
endif()
