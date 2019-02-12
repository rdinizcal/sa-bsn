CMAKE_MINIMUM_REQUIRED (VERSION 2.8)

PROJECT (libbsn)

###########################################################################
# Set the search path for .cmake files.
SET (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake.Modules" ${CMAKE_MODULE_PATH})

# Add a local CMake module search path dependent on the desired installation destination.
# Thus, artifacts from the complete source build can be given precendence over any installed versions.
IF(UNIX)
    SET (CMAKE_MODULE_PATH "${CMAKE_INSTALL_PREFIX}/share/cmake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules" ${CMAKE_MODULE_PATH})
ENDIF()
IF(WIN32)
    SET (CMAKE_MODULE_PATH "${CMAKE_INSTALL_PREFIX}/CMake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules" ${CMAKE_MODULE_PATH})
ENDIF()

###########################################################################
# Include flags for compiling.
INCLUDE (${CMAKE_CURRENT_SOURCE_DIR}/cmake.Modules/CompileFlags.cmake)

###########################################################################
# Find OpenDaVINCI.
SET(OPENDAVINCI_DIR "${CMAKE_INSTALL_PREFIX}")
FIND_PACKAGE (OpenDaVINCI REQUIRED)

###########################################################################
# Set linking libraries to successfully link test suites and binaries.
SET (LIBRARIES ${OPENDAVINCI_LIBRARIES})

IF(WIN32)
    SET (BSN_LIB bsn-static)
ELSE()
    SET (BSN_LIB bsn)
ENDIF()

###########################################################################
# Set header files from OpenDaVINCI.
INCLUDE_DIRECTORIES (${OPENDAVINCI_INCLUDE_DIRS})
# Set include directory.
INCLUDE_DIRECTORIES(include)

###########################################################################
# Collect all resource files.
FILE(GLOB_RECURSE libbsn-sources "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

###########################################################################
# Resulting artifacts.
ADD_LIBRARY (bsn-core   OBJECT ${libbsn-sources})
ADD_LIBRARY (bsn-static STATIC $<TARGET_OBJECTS:bsn-core>)
IF(NOT WIN32)
    ADD_LIBRARY (bsn    SHARED $<TARGET_OBJECTS:bsn-core>)
ENDIF()

TARGET_LINK_LIBRARIES (bsn-static ${LIBRARIES}) 
IF(NOT WIN32)
    TARGET_LINK_LIBRARIES (bsn ${LIBRARIES}) 
ENDIF()

###########################################################################
# Install this project.
INSTALL(TARGETS bsn-static DESTINATION lib COMPONENT lib)
IF(NOT WIN32)
    INSTALL(TARGETS bsn DESTINATION lib COMPONENT lib)
ENDIF()

# Install header files.
INSTALL(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/" DESTINATION include/bsn COMPONENT lib)

# Install CMake modules locally.
IF(UNIX)
    INSTALL(FILES "${CMAKE_CURRENT_SOURCE_DIR}/cmake.Modules/FindBSN.cmake" DESTINATION share/cmake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules COMPONENT lib)
    INSTALL(FILES "${CMAKE_CURRENT_SOURCE_DIR}/cmake.Modules/FindBSN.cmake" DESTINATION share/cmake-2.8/Modules COMPONENT lib)
    INSTALL(FILES "${CMAKE_CURRENT_SOURCE_DIR}/cmake.Modules/FindBSN.cmake" DESTINATION share/cmake-3.0/Modules COMPONENT lib)
ENDIF()
IF(WIN32)
    INSTALL(FILES "${CMAKE_CURRENT_SOURCE_DIR}/cmake.Modules/FindBSN.cmake" DESTINATION CMake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules COMPONENT lib)
ENDIF()