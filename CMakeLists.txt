cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set (PROJECT_NAME "stereoPoseEKF" CXX)
project(stereoPoseEKF)
# The version number.
set (stereoPoseEKF_VERSION_MAJOR 1)
set (stereoPoseEKF_VERSION_MINOR 0)

# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
  "${PROJECT_SOURCE_DIR}/src/stereoPoseEKF_viewerConfig.h.in"
  "${PROJECT_BINARY_DIR}/stereoPoseEKF_viewerConfig.h"
  )

# add the binary tree to the search path for include files
# so that we will find stereoPoseEKF_viewerConfig.h
include_directories("${PROJECT_BINARY_DIR}/src")
file(GLOB Dir1_Sources "${PROJECT_SOURCE_DIR}/src/*.cpp")

# boost
find_package( Boost REQUIRED )
include_directories( ${Boost_INCLUDE_DIRS} )

#Armadillo libraries
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

# PCL includes and libraries
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#OpenCV includes and libraries
find_package( OpenCV REQUIRED )
# INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )

# Final Executable
MESSAGE("PROJECT_SOURCE_DIR :" ${PROJECT_SOURCE_DIR})
MESSAGE("The cpp files are :" ${Dir1_Sources})
add_executable (stereoPoseEKF ${Dir1_Sources})
target_link_libraries (stereoPoseEKF ${PCL_LIBRARIES})
target_link_libraries( stereoPoseEKF ${OpenCV_LIBS} )
target_link_libraries (stereoPoseEKF ${Boost_LIBRARIES})
target_link_libraries (stereoPoseEKF ${ARMADILLO_LIBRARIES})
