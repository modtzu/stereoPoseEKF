cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set (PROJECT_NAME "stereoPoseEKF" CXX)
project(stereoPoseEKF)

SET (CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS_DISTRIBUTION "-Wall -Wextra -g")

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

# add_library(A OBJECT ${Dir1_Sources})
add_library(img2pcd OBJECT ${PROJECT_SOURCE_DIR}/src/img2pcd.cpp)
add_library(imgFtRelated OBJECT "${PROJECT_SOURCE_DIR}/src/imgFtRelated.cpp" )
add_library(featureManager OBJECT ${PROJECT_SOURCE_DIR}/src/featureManager.cpp)

add_library(input OBJECT ${PROJECT_SOURCE_DIR}/src/input.cpp)
add_library(poseEKF OBJECT "${PROJECT_SOURCE_DIR}/src/poseEKF.cpp" )
add_library(kltUncertainty OBJECT ${PROJECT_SOURCE_DIR}/src/kltUncertainty.cpp)

add_library(ppTransEst OBJECT ${PROJECT_SOURCE_DIR}/src/ppTransEst.cpp)
add_library(dq2omega OBJECT "${PROJECT_SOURCE_DIR}/src/dq2omega.cpp" )

add_library(utility OBJECT ${PROJECT_SOURCE_DIR}/src/utility.cpp)
add_library(keypointsManager OBJECT "${PROJECT_SOURCE_DIR}/src/keypointsManager.cpp" )
add_library(ppTransCov OBJECT ${PROJECT_SOURCE_DIR}/src/ppTransCov.cpp)

add_library(KltWithCov OBJECT ${PROJECT_SOURCE_DIR}/src/KltWithCov.cpp)
add_library(pclStereo OBJECT "${PROJECT_SOURCE_DIR}/src/pclStereo.cpp" )
add_library(stereoSolver OBJECT ${PROJECT_SOURCE_DIR}/src/stereoSolver.cpp)
add_library(pcVIsual OBJECT ${PROJECT_SOURCE_DIR}/src/pcVIsual.cpp)

SET(mytargets $<TARGET_OBJECTS:img2pcd> $<TARGET_OBJECTS:imgFtRelated> $<TARGET_OBJECTS:featureManager>)
SET(mytargets ${mytargets} $<TARGET_OBJECTS:input> $<TARGET_OBJECTS:poseEKF> $<TARGET_OBJECTS:kltUncertainty>)
SET(mytargets ${mytargets} $<TARGET_OBJECTS:ppTransEst> $<TARGET_OBJECTS:dq2omega> $<TARGET_OBJECTS:utility>)
SET(mytargets ${mytargets} $<TARGET_OBJECTS:keypointsManager> $<TARGET_OBJECTS:ppTransCov> $<TARGET_OBJECTS:KltWithCov>)
SET(mytargets ${mytargets} $<TARGET_OBJECTS:pclStereo> $<TARGET_OBJECTS:stereoSolver> $<TARGET_OBJECTS:pcVIsual>)

# add_library(input OBJECT ${PROJECT_BINARY_DIR}/src/input.cpp $<TARGET_OBJECTS:imgFtRelated>)
# target_link_libraries (input ${OpenCV_LIBS} )



# add_executable (stereoPoseEKF ${PROJECT_SOURCE_DIR}/src/main.cpp $<TARGET_OBJECTS:img2pcd> $<TARGET_OBJECTS:imgFtRelated> $<TARGET_OBJECTS:featureManager> $<TARGET_OBJECTS:input> $<TARGET_OBJECTS:poseEKF> $<TARGET_OBJECTS:kltUncertainty> $<TARGET_OBJECTS:ppTransEst> $<TARGET_OBJECTS:dq2omega> $<TARGET_OBJECTS:utility> $<TARGET_OBJECTS:keypointsManager> $<TARGET_OBJECTS:ppTransCov> $<TARGET_OBJECTS:KltWithCov> $<TARGET_OBJECTS:pclStereo> $<TARGET_OBJECTS:stereoSolver> $<TARGET_OBJECTS:pcVIsual>)
add_executable (stereoPoseEKF ${PROJECT_SOURCE_DIR}/src/main.cpp ${mytargets})
target_link_libraries (stereoPoseEKF ${PCL_LIBRARIES})
target_link_libraries( stereoPoseEKF ${OpenCV_LIBS} )
target_link_libraries (stereoPoseEKF ${Boost_LIBRARIES})
target_link_libraries (stereoPoseEKF ${ARMADILLO_LIBRARIES})


MESSAGE("object file :" ${mytargets})