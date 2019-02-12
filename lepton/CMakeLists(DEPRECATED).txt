CMAKE_MINIMUM_REQUIRED (VERSION 2.8)

PROJECT (lepton)

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
    SET (LEPTON_LIB lepton-static)
ELSE()
    SET (LEPTON_LIB lepton)
ENDIF()

###########################################################################
# Set header files from OpenDaVINCI.
INCLUDE_DIRECTORIES (${OPENDAVINCI_INCLUDE_DIRS})
# Set include directory.
INCLUDE_DIRECTORIES(include)

###########################################################################
# Collect all resource files.
FILE(GLOB_RECURSE lepton-sources "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

###########################################################################
# Resulting artifacts.
ADD_LIBRARY (lepton-core   OBJECT ${lepton-sources})
ADD_LIBRARY (lepton-static STATIC $<TARGET_OBJECTS:lepton-core>)
IF(NOT WIN32)
    ADD_LIBRARY (lepton    SHARED $<TARGET_OBJECTS:lepton-core>)
ENDIF()

TARGET_LINK_LIBRARIES (lepton-static ${LIBRARIES}) 
IF(NOT WIN32)
    TARGET_LINK_LIBRARIES (lepton ${LIBRARIES}) 
ENDIF()

###########################################################################
# Install this project.
INSTALL(TARGETS lepton-static DESTINATION lib COMPONENT lib)
IF(NOT WIN32)
    INSTALL(TARGETS lepton DESTINATION lib COMPONENT lib)
ENDIF()

# Install header files.
INSTALL(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/" DESTINATION include/lepton COMPONENT lib)

# Install CMake modules locally.
IF(UNIX)
    INSTALL(FILES "${CMAKE_CURRENT_SOURCE_DIR}/cmake.Modules/FindLEPTON.cmake" DESTINATION share/cmake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules COMPONENT lib)
    INSTALL(FILES "${CMAKE_CURRENT_SOURCE_DIR}/cmake.Modules/FindLEPTON.cmake" DESTINATION share/cmake-2.8/Modules COMPONENT lib)
    INSTALL(FILES "${CMAKE_CURRENT_SOURCE_DIR}/cmake.Modules/FindLEPTON.cmake" DESTINATION share/cmake-3.0/Modules COMPONENT lib)
ENDIF()
IF(WIN32)
    INSTALL(FILES "${CMAKE_CURRENT_SOURCE_DIR}/cmake.Modules/FindLEPTON.cmake" DESTINATION CMake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules COMPONENT lib)
ENDIF()